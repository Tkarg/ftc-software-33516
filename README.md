FTC Team 33516's code, available for public use since this is really not that good.

_____________________________________________________________________________________________________
Built based on RoadRunner, thus to use the code, Gradle needs reconfiguration.

Version 0.1.0:
  Class: 'ArtefactHandler'.
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
Classes also called alongside RoadRunner commands.
PIDF incorporated into motors to maintain target speeds.
Launcher and servo functions fixed.

Version 0.0.1:
  Classes: 'Intake' and 'Turret'.
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
Classes called alongside RoadRunner commands.
