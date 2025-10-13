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
            raise RuntimeError("piper non e' installato. Installa con: sudo apt-get install -y piper")
        if subprocess.call(['which', 'play'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
            raise RuntimeError("SoX non e' installato. Installa con: sudo apt-get install -y sox")

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
        # Riproduzione con lieve post processing
        subprocess.run([
            "play", wav_path, "--norm", "-q",
            "pitch", "400",      # +400 cent ~ +4 semitoni
            "tempo", "1.08",     # piu' veloce senza cambiare pitch
            "treble", "+3",      # un po' di brillantezza
            "highpass", "120"    # taglia le sub-basse
        ], check=True)


# ---------------------------
# Nodo ROS2 ASR + TTS (piper)
# ---------------------------
class ASRTTSNode(Node):
    def __init__(self):
        super().__init__('asr_tts_node')
        self.get_logger().info("Avvio nodo ASR+TTS (piper)")

        # --- Config ---
        config_path = '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/config/asr-tts.json'
        cfg = self._load_config(config_path)

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

        # ====== LOG DI AVVIO (INFO) ======
        self._log_config_start(config_path, cfg)

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

                    # Normalizza per cercare la wake-word
                    norm = raw_text if self.case_sensitive else raw_text.lower()
                    if self.normalize_accents:
                        norm = unicodedata.normalize('NFD', norm)
                        norm = ''.join(ch for ch in norm if unicodedata.category(ch) != 'Mn')

                    # TAGLIA SEMPRE tutto prima della wake-word, ovunque essa sia
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
                            # prendi testo dopo la wake-word sull'originale
                            start = idx + len(kw)
                            remainder = raw_text[start:].lstrip()
                            trig_word = kw
                            found = True
                            break

                    self.get_logger().info(f"Hai detto: {raw_text}")

                    if not found:
                        # nessuna wake-word trovata: ignora
                        if self.debug:
                            self.get_logger().debug("[DEBUG] Nessuna wake-word trovata; scarto risultato.")
                        continue

                    if self.debug:
                        self.get_logger().debug(f"[DEBUG] Wake matched: '{trig_word}' | remainder_raw='{remainder}'")

                    self._beep()
                    if remainder:
                        # pubblica SOLO la parte dopo la wake-word
                        self.publish_asr(remainder)
        except Exception as e:
            self.get_logger().error(f"Errore ascolto: {e}")

    # ----------------- Audio I/O -----------------
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
