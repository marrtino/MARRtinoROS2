#!/usr/bin/env python3
# Copyright 2025 robotics-3d.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com
# File  : asr_tts_node_piper.py

import os
import re
import json
import queue
import subprocess
import urllib.request
import zipfile
import unicodedata
import sounddevice as sd
from vosk import Model, KaldiRecognizer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------
# Utility: download Vosk model
# ---------------------------
def download_vosk_model(model_url: str, models_root: str):
    print(f"[VOSK] Scarico modello da {model_url}")
    zip_path = os.path.join(models_root, "vosk_model.zip")
    os.makedirs(models_root, exist_ok=True)
    try:
        urllib.request.urlretrieve(model_url, zip_path)
    except Exception as e:
        print(f"[VOSK] Errore download: {e}")
        return

    try:
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(models_root)
        os.remove(zip_path)
        print("[VOSK] Modello scaricato ed estratto.")
    except Exception as e:
        print(f"[VOSK] Errore estrazione: {e}")


# ---------------------------
# Piper TTS wrapper
# ---------------------------
class PiperTTS:
    """Wrapper per piper-tts via CLI."""
    SUPPORTED_VOICES = {
        "paola": "it_IT-paola-medium.onnx",
        "riccardo": "it_IT-riccardo-medium.onnx",
    }

    def __init__(
        self,
        models_dir: str,
        default_voice: str = "paola",
        length_scale: float = 1.0,
        noise_scale: float = 0.667,
        noise_w: float = 0.8,
    ):
        self.models_dir = models_dir
        self.voice = default_voice if default_voice in self.SUPPORTED_VOICES else "paola"
        self.length_scale = str(length_scale)
        self.noise_scale = str(noise_scale)
        self.noise_w = str(noise_w)

        # Verifica dipendenze
        if subprocess.call(['which', 'piper'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
            raise RuntimeError("piper non e installato. Installa con: sudo apt-get install -y piper")
        if subprocess.call(['which', 'play'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
            raise RuntimeError("SoX non e installato. Installa con: sudo apt-get install -y sox")

    def set_voice(self, voice_name: str) -> bool:
        v = (voice_name or "").strip().lower()
        if v in self.SUPPORTED_VOICES:
            self.voice = v
            return True
        return False

    def _model_path(self) -> str:
        return os.path.join(self.models_dir, self.SUPPORTED_VOICES[self.voice])

    def get_params(self):
        return {
            "voice": self.voice,
            "model_path": self._model_path(),
            "length_scale": self.length_scale,
            "noise_scale": self.noise_scale,
            "noise_w": self.noise_w
        }

    def speak(self, text: str, wav_path: str = "/tmp/robot_speech.wav"):
        if not text:
            return
        model = self._model_path()
        if not os.path.exists(model):
            raise FileNotFoundError(f"Modello piper non trovato: {model}")
        # Sintesi
        subprocess.run(
            [
                "piper",
                "--model", model,
                "--output_file", wav_path,
                "--length_scale", self.length_scale,
                "--noise_scale", self.noise_scale,
                "--noise_w", self.noise_w,
            ],
            input=text.encode("utf-8"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
        # Riproduzione con leggera post-EQ/tempo
        subprocess.run([
            "play", wav_path, "--norm", "-q",
            "pitch", "400",      # +400 cent e +4 semitoni
            "tempo", "0.9",     # pie veloce senza cambiare pitch
            "treble", "+3",      # brillantezza
            "highpass", "120"    # taglia le sub-basse
        ], check=True)


# ---------------------------
# Nodo ROS2 ASR + TTS (piper)
# ---------------------------
class ASRTTSNode(Node):
    def __init__(self):
        super().__init__('asr_tts_node_piper')
        self.get_logger().info("Avvio nodo ASR+TTS (piper)")

        # --- Config ---
        config_path = '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/config/asr-tts.json'
        cfg = self._load_config(config_path)
        config_dir = os.path.dirname(config_path)

        # Debug level
        self.debug = cfg.get("debug", True)
        try:
            if self.debug:
                from rclpy.logging import LoggingSeverity
                self.get_logger().set_level(LoggingSeverity.DEBUG)
        except Exception:
            pass

        # Audio base
        self.output_samplerate = cfg["output_samplerate"]
        self.output_device_index = cfg["output_device_index"]
        self.input_samplerate = cfg["input_samplerate"]

        # Lingua / messaggi
        self.language = cfg.get("language", "it")
        self.work_offline = cfg.get("work_offline", True)
        self.msg_start = cfg.get("msg_start", "Ciao, sono pronta!")

        # Flag normalizzazione (prima di _resolve_wake_words)
        self.case_sensitive = cfg.get("case_sensitive", False)
        self.normalize_accents = cfg.get("normalize_accents", True)
        self.wake_match = cfg.get("wake_match", "prefix").lower()

        # Wake words
        self.wake_words = self._resolve_wake_words(cfg)

        # Beep config
        self.beep_enabled = cfg.get("beep", {}).get("enabled", True)
        self.beep_freq = cfg.get("beep", {}).get("frequency", 1000)
        self.beep_ms = cfg.get("beep", {}).get("duration_ms", 180)
        self.beep_volume = cfg.get("beep", {}).get("volume", 0.2)

        # Piper config
        self.piper_models_dir = cfg.get(
            "piper_models_dir",
            "/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/models/piper"
        )
        self.piper_default_voice = cfg.get("piper_default_voice", "paola")
        self.piper_length_scale = cfg.get("piper_length_scale", 1.0)
        self.piper_noise_scale = cfg.get("piper_noise_scale", 0.667)
        self.piper_noise_w = cfg.get("piper_noise_w", 0.8)

        # NLP correzioni (da file JSON esterno in config/)
        self.nlp_corrections = cfg.get("nlp_corrections", True)
        vocab_filename = cfg.get("commands_vocab_file", "commands_vocab.it.json")
        vocab_path = vocab_filename if os.path.isabs(vocab_filename) else os.path.join(config_dir, vocab_filename)
        self.commands_vocab = self._load_commands_vocab(vocab_path)

        # ====== LOG DI AVVIO (INFO) ======
        self._log_config_start(config_path, cfg)
        self.get_logger().info(f"commands_vocab_file: {vocab_path} (items={len(self.commands_vocab)})")

        # Microfono ReSpeaker
        device_list = sd.query_devices()
        self.input_device_index = None
        for idx, device in enumerate(device_list):
            name = str(device.get('name', ''))
            if 'ReSpeaker' in name:
                self.input_device_index = idx
                break
        if self.input_device_index is None:
            raise RuntimeError("Microfono ReSpeaker non trovato")

        # ASR (Vosk)
        self.model_vosk_root = "/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/models"
        self.vosk_model_dir = os.path.join(self.model_vosk_root, "vosk-model-it-0.22")
        self.vosk_model_url = "https://alphacephei.com/vosk/models/vosk-model-it-0.22.zip"
        expected_file = os.path.join(self.vosk_model_dir, "am", "final.mdl")
        if not os.path.exists(expected_file):
            download_vosk_model(self.vosk_model_url, self.model_vosk_root)
        self.model_vosk = Model(self.vosk_model_dir)

        # Publisher / Subscriber
        self.publisher_asr = self.create_publisher(String, '/ASR', 10)
        self.subscription_lang = self.create_subscription(String, '/speech/language', self.language_callback, 10)
        self.subscription_text = self.create_subscription(String, '/speech/to_speak', self.tts_callback, 10)
        self.subscription_voice = self.create_subscription(String, '/speech/voice', self.voice_callback, 10)

        # Social topics
        self.TOPIC_emotion = "social/emotion"
        self.TOPIC_gesture = "social/gesture"
        self.emotion_pub = self.create_publisher(String, self.TOPIC_emotion, 10)
        self.gesture_pub = self.create_publisher(String, self.TOPIC_gesture, 10)

        # Stato
        self.listening = True
        self.tts_busy = False
        self.queue = queue.Queue()
        self.recognizer = KaldiRecognizer(self.model_vosk, self.input_samplerate)

        # Dispositivi audio
        sd.default.device = (self.input_device_index, self.output_device_index)

        # Piper TTS
        self.piper = PiperTTS(
            models_dir=self.piper_models_dir,
            default_voice=self.piper_default_voice,
            length_scale=self.piper_length_scale,
            noise_scale=self.piper_noise_scale,
            noise_w=self.piper_noise_w,
        )
        if self.debug:
            p = self.piper.get_params()
            self.get_logger().debug(
                f"[DEBUG] Piper init -> voice={p['voice']} | model={p['model_path']} | "
                f"length_scale={p['length_scale']} | noise_scale={p['noise_scale']} | noise_w={p['noise_w']}"
            )

        # Avvio microfono
        self.stream = sd.RawInputStream(
            samplerate=self.input_samplerate,
            dtype='int16',
            channels=1,
            callback=self.audio_callback,
            device=self.input_device_index
        )
        self.stream.start()
        self.get_logger().info("Microfono avviato")

        # Loop ascolto
        self.create_timer(0.1, self.listen_loop)
        self.get_logger().info("Timer ascolto continuo avviato.")

        # Saluto iniziale
        self.emotion("startblinking")
        self.emotion("speak")
        self.speak(self.msg_start)
        self.emotion("normal")

    # ----------------- LOG CONFIG (INFO) -----------------
    def _log_config_start(self, config_path: str, cfg: dict):
        self.get_logger().info("=== START CONFIG ===")
        self.get_logger().info(f"config_path: {config_path}")
        self.get_logger().info(f"language: {self.language}")
        self.get_logger().info(f"input_samplerate: {self.input_samplerate}")
        self.get_logger().info(f"output_samplerate: {self.output_samplerate}")
        self.get_logger().info(f"output_device_index: {self.output_device_index}")
        self.get_logger().info(f"wake_words: {self.wake_words}")
        self.get_logger().info(f"wake_match: {self.wake_match}")
        self.get_logger().info(f"case_sensitive: {self.case_sensitive}")
        self.get_logger().info(f"normalize_accents: {self.normalize_accents}")
        self.get_logger().info(f"beep.enabled: {self.beep_enabled}")
        self.get_logger().info(f"beep.frequency: {self.beep_freq}")
        self.get_logger().info(f"beep.duration_ms: {self.beep_ms}")
        self.get_logger().info(f"beep.volume: {self.beep_volume}")
        self.get_logger().info(f"piper_models_dir: {self.piper_models_dir}")
        self.get_logger().info(f"piper_default_voice: {self.piper_default_voice}")
        self.get_logger().info(f"piper_length_scale: {self.piper_length_scale}")
        self.get_logger().info(f"piper_noise_scale: {self.piper_noise_scale}")
        self.get_logger().info(f"piper_noise_w: {self.piper_noise_w}")
        self.get_logger().info(f"debug: {self.debug}")
        self.get_logger().info("====================")
        if cfg.get("debug", False):
            self.get_logger().debug(f"[DEBUG] Config completa:\n{json.dumps(cfg, indent=2, ensure_ascii=False)}")

    # ----------------- Config helpers -----------------
    def _load_config(self, path: str) -> dict:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Config mancante: {path}")
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _load_commands_vocab(self, path: str):
        try:
            if not os.path.exists(path):
                self.get_logger().warning(f"[NLP] File vocabolario comandi non trovato: {path}")
                return []
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if not isinstance(data, list):
                self.get_logger().warning(f"[NLP] Formato non valido in {path}: attesa una lista di oggetti")
                return []
            # sanity check elementi
            cleaned = []
            for it in data:
                if isinstance(it, dict) and "phrase" in it:
                    cleaned.append({
                        "phrase": str(it["phrase"]),
                        "intent": str(it.get("intent", it["phrase"]))
                    })
            return cleaned
        except Exception as e:
            self.get_logger().warning(f"[NLP] Impossibile caricare vocabolario {path}: {e}")
            return []

    def _resolve_wake_words(self, cfg: dict):
        if "wake_words" in cfg and isinstance(cfg["wake_words"], list) and cfg["wake_words"]:
            words = [str(w) for w in cfg["wake_words"] if str(w).strip()]
        else:
            words = [cfg.get("wake_word", "martino")]
        normed = []
        for w in words:
            nw = w if self.case_sensitive else w.lower()
            if self.normalize_accents:
                nw = unicodedata.normalize('NFD', nw)
                nw = ''.join(ch for ch in nw if unicodedata.category(ch) != 'Mn')
            normed.append(nw)
        return normed

    # ----------------- NLP helpers -----------------
    def _levenshtein(self, a: str, b: str) -> int:
        if a == b:
            return 0
        if len(a) == 0:
            return len(b)
        if len(b) == 0:
            return len(a)
        v0 = list(range(len(b) + 1))
        v1 = [0] * (len(b) + 1)
        for i in range(len(a)):
            v1[0] = i + 1
            for j in range(len(b)):
                cost = 0 if a[i] == b[j] else 1
                v1[j + 1] = min(v1[j] + 1, v0[j + 1] + 1, v0[j] + cost)
            v0, v1 = v1, v0
        return v0[len(b)]

    def _norm_basic(self, text: str) -> str:
        t = text.strip().lower()
        if self.normalize_accents:
            t = unicodedata.normalize('NFD', t)
            t = ''.join(ch for ch in t if unicodedata.category(ch) != 'Mn')
        t = re.sub(r"[^\w\s']", " ", t)
        t = re.sub(r"\s+", " ", t).strip()
        return t

    def _snap_to_vocab(self, text: str) -> str:
        if not getattr(self, "nlp_corrections", True):
            return text
        vocab = getattr(self, "commands_vocab", [])
        if not vocab:
            return text
        best = None
        best_score = -1.0
        for item in vocab:
            phrase = item.get("phrase", "")
            intent = item.get("intent", phrase)
            if not phrase:
                continue
            dist = self._levenshtein(text, phrase)
            maxlen = max(len(text), len(phrase), 1)
            score = 1.0 - (dist / maxlen)  # 1 perfetto, 0 pessimo
            if score > best_score:
                best_score = score
                best = intent
        return best if best_score >= 0.72 else text  # soglia empirica

    def _postprocess_text(self, text: str) -> str:
        t = self._norm_basic(text)
        t = self._snap_to_vocab(t)
        return t

    # ----------------- Social helpers -----------------
    def gesture(self, msg: str):
        self.gesture_pub.publish(String(data=msg))

    def emotion(self, msg: str):
        self.emotion_pub.publish(String(data=msg))

    # ----------------- Callbacks -----------------
    def language_callback(self, msg: String):
        self.language = msg.data
        self.get_logger().info(f"Lingua impostata: {self.language}")

    def voice_callback(self, msg: String):
        requested = (msg.data or "").strip().lower()
        if self.piper.set_voice(requested):
            self.get_logger().info(f"Voce piper cambiata in: {requested}")
        else:
            self.get_logger().warning("Voce non supportata. Usa 'paola' o 'riccardo'.")

    def tts_callback(self, msg: String):
        self._speak_common(msg.data)

    # ----------------- Beep -----------------
    def _beep(self):
        if not self.beep_enabled:
            return
        try:
            dur = max(0.05, min(1.0, self.beep_ms / 1000.0))
            vol = str(max(0.0, min(1.0, float(self.beep_volume))))
            freq = str(int(self.beep_freq))
            subprocess.Popen(
                ["play", "-nq", "synth", str(dur), "sin", freq, "vol", vol],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().warning(f"Beep fallito: {e}")

    # ----------------- TTS -----------------
    def speak(self, text: str):
        self._speak_common(text)

    def _speak_common(self, text: str):
        if not text:
            return
        self.tts_busy = True
        self.listening = False
        try:
            self.stream.stop()
            with self.queue.mutex:
                self.queue.queue.clear()
        except Exception:
            pass
        try:
            if self.debug:
                p = self.piper.get_params()
                self.get_logger().debug(
                    f"[DEBUG] Piper speak -> voice={p['voice']} | model={p['model_path']} | "
                    f"length_scale={p['length_scale']} | noise_scale={p['noise_scale']} | "
                    f"noise_w={p['noise_w']} | text_len={len(text)}"
                )
            self.emotion("speak")
            self.piper.speak(text, "/tmp/robot_speech.wav")
        except Exception as e:
            self.get_logger().error(f"Errore TTS (piper): {e}")
        try:
            self.stream.start()
        except Exception:
            pass
        self.emotion("normal")
        self.tts_busy = False
        self.listening = True

    # ----------------- ASCOLTO -----------------
    def listen_loop(self):
        if not self.listening or self.tts_busy:
            return
        try:
            while not self.queue.empty():
                data = self.queue.get_nowait()
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    raw_text = (result.get("text", "") or "").strip()
                    if not raw_text:
                        continue

                    # normalizza per la ricerca wake
                    norm = raw_text if self.case_sensitive else raw_text.lower()
                    if self.normalize_accents:
                        norm = unicodedata.normalize('NFD', norm)
                        norm = ''.join(ch for ch in norm if unicodedata.category(ch) != 'Mn')

                    # CERCA LA WAKE-WORD OVUNQUE (indipendente da wake_match)
                    found = False
                    remainder = ""
                    trig_word = ""
                    for kw in self.wake_words:
                        nkw = kw if self.case_sensitive else kw.lower()
                        if self.normalize_accents:
                            nkw = unicodedata.normalize('NFD', nkw)
                            nkw = ''.join(ch for ch in nkw if unicodedata.category(ch) != 'Mn')
                        idx = norm.find(nkw)
                        if idx != -1:
                            start = idx + len(kw)  # taglia su raw_text dopo la wake-word
                            remainder = raw_text[start:].lstrip()
                            trig_word = kw
                            found = True
                            break

                    self.get_logger().info(f"Hai detto: {raw_text}")

                    if not found:
                        # nessuna wake-word trovata -> non beep
                        continue

                    # BEEP SEMPRE AL RILEVAMENTO
                    self._beep()

                    # Se c'è del testo dopo la wake-word, pubblicalo (eventuale correzione NLP)
                    if remainder:
                        corrected = self._postprocess_text(remainder)
                        if self.debug and corrected != remainder:
                            self.get_logger().debug(f"[DEBUG] NLP corrected -> '{corrected}' (from '{remainder}')")
                        self.publish_asr(corrected)
                        self.speak("o capito un attimo e ti rispondo")

                # if self.recognizer.AcceptWaveform(data):
                #     result = json.loads(self.recognizer.Result())
                #     raw_text = (result.get("text", "") or "").strip()
                #     if not raw_text:
                #         continue
                #     match_text = raw_text if self.case_sensitive else raw_text.lower()
                #     if self.normalize_accents:
                #         match_text = unicodedata.normalize('NFD', match_text)
                #         match_text = ''.join(ch for ch in match_text if unicodedata.category(ch) != 'Mn')
                #     self.get_logger().info(f"Hai detto: {raw_text}")
                #     triggered, trig_word, remainder = self._check_wake(match_text, raw_text)
                #     if triggered:
                #         if self.debug:
                #             self.get_logger().debug(f"[DEBUG] Wake matched: '{trig_word}' | remainder_raw='{remainder}'")
                #         self._beep()
                #         if remainder:
                #             corrected = self._postprocess_text(remainder)
                #             if self.debug and corrected != remainder:
                #                 self.get_logger().debug(f"[DEBUG] NLP corrected -> '{corrected}' (from '{remainder}')")
                #             self.publish_asr(corrected)
        except Exception as e:
            self.get_logger().error(f"Errore ascolto: {e}")

    def _check_wake(self, match_text: str, raw_text_original: str):
        # anywhere: ignora qualsiasi cosa prima della wake-word, prendi solo il testo dopo
        if self.wake_match == "anywhere":
            for kw in self.wake_words:
                idx = match_text.find(kw)
                if idx != -1:
                    after = raw_text_original[idx + len(kw):].lstrip()
                    return True, kw, after.strip()
            return False, "", ""
        else:  # prefix
            for kw in self.wake_words:
                if match_text.startswith(kw):
                    remainder = raw_text_original[len(kw):].lstrip()
                    return True, kw, remainder
            return False, "", ""

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(str(status))
        self.queue.put(bytes(indata))

    def publish_asr(self, text: str):
        self.publisher_asr.publish(String(data=text))


# ------------- main -------------
def main(args=None):
    rclpy.init(args=args)
    node = ASRTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
