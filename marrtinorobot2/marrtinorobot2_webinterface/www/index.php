<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="utf-8">
  <title>MARRtino Social User Interface</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">

  <!-- Bootstrap CSS -->
  <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css">

  <!-- Script -->
  <script src="js/jquery-3.4.1.min.js"></script>
  <script src="bootstrap/js/bootstrap.min.js"></script>
  <script src="js/roslib.min.js"></script>
  <script src="js/eventemitter2.min.js"></script>
  <script src="js/keyboardteleop.min.js"></script>

  <script type="text/javascript">
    var teleop;
    var ros = new ROSLIB.Ros({
      url: 'ws:' + window.location.hostname + ':9090'
    });

    ros.on('connection', function () {
      document.getElementById("status").innerHTML = "Connected";
    });

    ros.on('error', function (error) {
      document.getElementById("status").innerHTML = "Error";
    });

    ros.on('close', function () {
      document.getElementById("status").innerHTML = "Closed";
    });

    window.onload = function () {
      // Eventuali inizializzazioni
    }
  </script>

  <style>
    .container {
      position: relative;
    }

    h6 {
      text-align: center;
    }

    .card img {
      margin: 0 auto;
    }

    .btn {
      width: 100%;
    }
  </style>
</head>

<body>
  <?php include "nav.php"; ?>

  <div class="container">
    <!-- Logo Row -->
    <!-- <div class="row">
      <div class="col-md-1"></div>
      <div class="col-md-4">
        <div class="thumbnail">
          <a href="https://www.robotics-3d.com">
            <img style="display: block; margin: auto;" src="image/LogoRobotics.png" alt="robotics-3d.com" width="40%">
          </a>
        </div>
      </div>
      <div class="col-md-2"></div>
      <div class="col-md-4">
        <div class="thumbnail">
          <a href="https://robotics.surfweb.eu" target="_blank">
            <img style="display: block; margin: auto;" src="image/LogoCircolareATNero.png" alt="artigianitecnologici.it" width="40%">
          </a>
        </div>
      </div>
      <div class="col-md-1"></div>
    </div> -->
    <h6>Interface version 2.1</h6>

    <!-- Cards Section /bringup/bringup.php"-->
    <div class="row">
      <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <a href="../bringup/bringup.php">
            <img class="card-img-top" src="images/bringup.webp" alt="bringup">
          </a>
          <div class="card-body">
            <div class="d-flex justify-content-center">
              <a class="btn btn-outline-danger" href="../bringup/bringup.php" role="button">BRING UP</a>
            </div>
          </div>
        </div>
      </div>

      <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <a href="social/facerobot.php">
            <img class="card-img-top" src="images/social.webp" alt="Marrtina">
          </a>
          <div class="card-body">
            <div class="d-flex justify-content-center">
              <a class="btn btn-outline-danger" href="social/facerobot.php" role="button">Social Interface</a>
            </div>
          </div>
        </div>
      </div>

       <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <script>
            document.write("<a href='http://" + window.location.hostname + ":5000' target='_blank'>");
            document.write("<img class='card-img-top' src='images/eva.png' alt='E.V.A. Interface'>");
            document.write("</a>");
          </script>
          <div class="card-body">
            <div class="d-flex justify-content-center">
               <script>
                  document.write("<a class='btn btn-outline-danger' href='http://" + window.location.hostname + ":5000' target='_blank'>E.V.A. Interface</a>");
               </script>
                  </div>
          </div>
        </div>
      </div>

      <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <script>
            document.write("<a href='http://" + window.location.hostname + ":8085' target='_blank'>");
            document.write("<img class='card-img-top' src='images/vnc.webp' alt='VNC Interface'>");
            document.write("</a>");
          </script>
          <div class="card-body">
            <div class="d-flex justify-content-center">
              <script>
                document.write("<a class='btn btn-outline-danger' href='http://" + window.location.hostname + ":8085' target='_blank'>VNC Interface</a>");
              </script>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Manual -->
    <!-- <div class="row">
      <div class="col-md-3"></div>
      <div class="col-md-6 text-center">
        <a class="btn btn-success btn-lg" href="https://social.marrtino.org" target="_blank" role="button">MANUALE MARRtino SOCIAL ROBOT</a>
      </div>
      <div class="col-md-3"></div>
    </div> -->

    <!-- Blockly -->
    <div class="row">
      <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <a href="../program/blockly_robot.php">
            <img class="card-img-top" src="images/pyr.webp" alt="Blockly Program Interface">
          </a>
          <div class="card-body">
            <div class="d-flex justify-content-center">
              <a class="btn btn-outline-danger" href="../program/blockly_robot.php" role="button">Blockly </a>
              <a class="btn btn-outline-danger" href="../program/python_robot.php" role="button">Python 3</a>
            </div>
            <div class="d-flex justify-content-center">
              <a class="btn btn-outline-danger" href="../program/tile_robot.php" role="button">Tile</a>
              
            </div>
          </div>
        </div>
      </div>

      <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <a href="marrtina.html">
            <img class="card-img-top" src="images/telepresence.webp" alt="Marrtina">
          </a>
          <div class="card-body">
            <div class="d-flex justify-content-center">
              <a class="btn btn-outline-danger" href="navigation.php" role="button">TELEPRESENCE</a>
            </div>
          </div>
        </div>
      </div>

      <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <img class="card-img-top" src="images/demo.webp" alt="Card image cap">
          <div class="card-body">
            <div class="d-flex justify-content-center">
              <a class="btn btn-outline-danger" href="../social/demorobotics.php" role="button">DEMO</a>
            </div>
          </div>
        </div>
      </div>

      <!-- <div class="col-md-3">
        <div class="card mb-4 box-shadow">
          <script>
            document.write("<a href='https://" + window.location.hostname + ":9200' target='_blank'>");
            document.write("<img class='card-img-top' src='images/ssh.webp' alt='SSH'>");
            document.write("</a>");
          </script>
          <div class="card-body">
            <div class="d-flex justify-content-center">
              <script>
                document.write("<a class='btn btn-outline-danger' href='https://" + window.location.hostname + ":9200' target='_blank'>SSH Interface</a>");
              </script>
            </div>
          </div>
        </div>
      </div> -->
    </div>
  </div>
</body>

</html>
