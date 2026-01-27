FTC Team <strong>33516</strong>'s code, available for public use since this is really not that good.

<hr>
Built based on <i>RoadRunner</i>, thus to use the code, <mark>Gradle needs reconfiguration</mark>.

<h1>Version 0.1.0.</h1>
  <pre>
   +---Class ArtefactHandler:
          +-function takeArtefact:
          |    Power intake motors to take in artefact while spinning launching flywheel backwards.
          +-function keepArtefact:
          |    Stop all intake motors.
          +-function discardArtefact:
          |    Spin intake motors backwards.
          +-function spoolUp:
          |    Power up launcher motors until reaching set velocity.
          +-function launchArtefact:
          |    Power up intake motors while launcher motors run.
          +-function Halt:
               Stop all motors involved in launching artefacts.
  </pre>
<br>
Classes also called alongside RoadRunner commands.<br>
PIDF incorporated into motors to maintain target speeds.<br>
Launcher and servo functions fixed.

<hr>

<h1>Version 0.0.1.</h1>
  <pre>
   +---Class Intake:
   |      +-function takeArtefact:
   |      |     Power intake motors to take in artefact while spinning launching flywheel backwards.
   |      +-function keepArtefact:
   |      |     Stop all intake motors.
   |      +-function discardArtefact:
   |            Spin intake motors backwards.
   +---Class Turret:
          +-function spoolUp:
          |    Power up launcher motors until they reach set velocity.
          +-function launchArtefact:
          |    Power up intake motors while launcher motors are running.
          +-function Halt:
               Stop all motors involved in launching artefacts.
  </pre>
<br>
Classes called alongside RoadRunner commands.<br>

<hr>
