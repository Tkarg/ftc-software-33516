package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

/*
 * Before anything, we have a few notes:
 *
 * Positive direction is clockwise from the perspective opposite to the motor axially.
 *
 * We are using REV 41 and GoBilda 5203 for the launcher because we are out of resources.
 *
 * Both motor types count 28 ticks per revolution.
 *
 * Latest measurement gives 12150 ticks per 23.8125 in, for our tiles are not perfect.
 * */

@Autonomous(name = "A_RED_N")
public class _AutonomousRedNear_ extends LinearOpMode{

    public static class ArtefactHandler{

        private final DcMotorEx REVLauncherR, REVLauncherL;
        private final DcMotor REVLoader, GoBildaIntake;
        Servo Aimer;
        double LaunchSpeed = 2500;

        /*
         * Turns out the ideal launch speed for AUTO is going to be 2500RPM.
         * */

        public double TPR = 28;

        //Overclocking time!
        PIDFCoefficients PIDF = new PIDFCoefficients(1000000, 1, 0, 3100);
        /*
         * We have quite a beefy flywheel doing the launching job here, so it's really that logical to step P up to 1000000.
         * For F it is fine at 19, since we had had enough tuning for the day.
         * I is 1 to make the steady error of roughly 100 to 150 RPM lower.
         * D is out of the question, it is just way too risky to erode our transmission belts.
         * Motors in use: REV HD 40:1, max speed 6000RPM, all 28 ticks per revolution.
         * Motors are expected to get really really hot quickly, because the flywheel has too much inertia to stop immediately.
         * Launcher also uses the hood, a servo attached to a little curve that will give the ball some more horizontal speed.
         * Air drag makes trajectory really unpredictable so really it is rather futile to map the paths.
         * Thus for now, all is guesswork.
         * */

        private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        public ArtefactHandler (HardwareMap hardwareMap){

            //TURRET.

            REVLauncherR = hardwareMap.get(DcMotorEx.class, "REVR");
            REVLauncherR.setDirection(DcMotorSimple.Direction.FORWARD);
            REVLauncherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

            REVLauncherL = hardwareMap.get(DcMotorEx.class, "REVL");
            REVLauncherL.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLauncherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

            //INTAKE.

            GoBildaIntake = hardwareMap.get(DcMotor.class, "GoBildaIntake");
            GoBildaIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            GoBildaIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            REVLoader = hardwareMap.get(DcMotor.class, "REVTransfer");
            REVLoader.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //AIM ASSIST.

            Aimer = hardwareMap.get(Servo.class, "Hood");
            Aimer.setPosition(0);
        }

        /*
         * Both GoBilda and REV have their motors running on 28 ticks each revolution.
         * Desired speed in rotation per minute will be divided by 60 to get per second rotation.
         * Multiply that with 28 we have the desired angular speed.
         * */

        //INTAKE.

        public class Take_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBildaIntake.setPower(1);
                REVLoader.setPower(0.3);
                REVLauncherL.setVelocity((-1100.0 / 60) * TPR);
                REVLauncherR.setVelocity((-1100.0 / 60) * TPR);
                return false;
            }
        }
        public Action takeArtefact(){
            return new Take_Artefact();
        }

        public class Discard_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBildaIntake.setPower(-1);
                REVLoader.setPower(-1);
                return false;
            }
        }
        public Action discardArtefact(){
            return new Discard_Artefact();
        }

        public class Keep_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                REVLauncherL.setVelocity(0);
                REVLauncherR.setVelocity(0);
                return false;
            }
        }
        public Action keepArtefact(){
            return new Keep_Artefact();
        }

        //TURRET.

        public class Spool_Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                Aimer.setPosition(0);
                //Under voltage prevention.
                REVLauncherR.setVelocity((LaunchSpeed / 60) * TPR);
                REVLauncherL.setVelocity((LaunchSpeed / 60) * TPR);
                //We need real speed.
                return (REVLauncherR.getVelocity() + REVLauncherL.getVelocity()) / 2 < (LaunchSpeed / 60) * TPR;
            }
        }
        public Action spoolUp() {
            return new Spool_Up();
        }

        public class Launch_Artefact implements Action {
            private boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                if (!init) {
                    timer.reset();
                    init = true;
                }
                if (timer.time(TimeUnit.MILLISECONDS) < 300 |
                        (timer.time(TimeUnit.MILLISECONDS) > 600 & timer.time(TimeUnit.MILLISECONDS) < 900) |
                        (timer.time(TimeUnit.MILLISECONDS) > 1200 & timer.time(TimeUnit.MILLISECONDS) < 1500)) {
                    GoBildaIntake.setPower(1);
                    REVLoader.setPower(1);
                } else {
                    GoBildaIntake.setPower(0);
                    REVLoader.setPower(0);
                }
                return timer.time(TimeUnit.MILLISECONDS) < 1300;
            }}

        public Action launchArtefact() {
            return new Launch_Artefact();
        }

        public class Halt implements Action {
            private boolean init = false;
            @Override
            public boolean run (@NonNull TelemetryPacket telemetry){
                if (!init) {
                    timer.reset();
                    init = true;
                }
                REVLauncherR.setVelocity(0);
                REVLauncherL.setVelocity(0);
                REVLauncherR.setPower(0);
                REVLauncherL.setPower(0);
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                return timer.time(TimeUnit.MILLISECONDS) < 500;
            }
        }

        public Action halt() {
            return new Halt();
        }

    }

    //PATH.

    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d Location = new Pose2d(47.3, 47.3, Math.toRadians(-51));
        MecanumDrive Base = new MecanumDrive(hardwareMap, Location);
        ArtefactHandler artefactHandler = new ArtefactHandler(hardwareMap);
        waitForStart();
        //Why am I doing this?
        Action Path1 = Base.actionBuilder(Location)
                .strafeToSplineHeading(new Vector2d(-24, 24), Math.toRadians(-225))
                .build();
        Action Path2 = Base.actionBuilder(new Pose2d(-24, 24, Math.toRadians(-225)))
                .strafeToSplineHeading(new Vector2d(-12, 20), Math.toRadians(90))
                .build();
        Action Path3 = Base.actionBuilder(new Pose2d(-12, 20, Math.toRadians(90)))
                .strafeTo(new Vector2d(-12, 52))
                .build();
        Action Path4 = Base.actionBuilder(new Pose2d(-12, 52, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-24, 24), Math.toRadians(-225))
                .build();
        Action Path5 = Base.actionBuilder(new Pose2d(-24, 24, Math.toRadians(-225)))
                .strafeToSplineHeading(new Vector2d(12, 20), Math.toRadians(90))
                .build();
        Action Path6 = Base.actionBuilder(new Pose2d(12, 20, Math.toRadians(90)))
                .strafeTo(new Vector2d(12, 52))
                .build();
        Action Path7 = Base.actionBuilder(new Pose2d(12, 52, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-24, 24), Math.toRadians(-225))
                .build();
        Action Path8 = Base.actionBuilder(new Pose2d(-24, 24, Math.toRadians(-225)))
                .strafeToSplineHeading(new Vector2d(36, 20), Math.toRadians(90))
                .build();
        Action Path9 = Base.actionBuilder(new Pose2d(36, 20, Math.toRadians(90)))
                .strafeTo(new Vector2d(36, 52))
                .build();
        Action PathA = Base.actionBuilder(new Pose2d(36, 52, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-24, 24), Math.toRadians(-225))
                .build();
        Action PathB = Base.actionBuilder(new Pose2d(-24, 24, Math.toRadians(-225)))
                .strafeToSplineHeading(new Vector2d(0, 24), Math.toRadians(0))
                .build();

        ParallelAction Spool1 = new ParallelAction(
                Path1,
                artefactHandler.spoolUp()
        );
        ParallelAction Defer2 = new ParallelAction(
                Path2,
                artefactHandler.discardArtefact()
        );
        ParallelAction Refer3 = new ParallelAction(
                Path3,
                artefactHandler.takeArtefact()
        );
        ParallelAction Spool4 = new ParallelAction(
                Path4,
                artefactHandler.spoolUp(),
                artefactHandler.keepArtefact()
        );
        ParallelAction Defer5 = new ParallelAction(
                Path5,
                artefactHandler.discardArtefact()
        );
        ParallelAction Refer6 = new ParallelAction(
                Path6,
                artefactHandler.takeArtefact()
        );
        ParallelAction Spool7 = new ParallelAction(
                Path7,
                artefactHandler.spoolUp(),
                artefactHandler.keepArtefact()
        );
        ParallelAction Defer8 = new ParallelAction(
                Path8,
                artefactHandler.discardArtefact()
        );
        ParallelAction Refer9 = new ParallelAction(
                Path9,
                artefactHandler.takeArtefact()
        );
        ParallelAction SpoolA = new ParallelAction(
                PathA,
                artefactHandler.spoolUp(),
                artefactHandler.keepArtefact()
        );

        SequentialAction Red = new SequentialAction(
                Spool1,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                Defer2,
                Refer3,
                artefactHandler.keepArtefact(),
                Spool4,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                Defer5,
                Refer6,
                artefactHandler.keepArtefact(),
                Spool7,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                Defer8,
                Refer9,
                artefactHandler.keepArtefact(),
                SpoolA,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                PathB
        );

        Actions.runBlocking(Red);
    }
}

//ENDING POSITION: (+00;+24); HEADING 0.

