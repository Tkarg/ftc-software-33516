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
*
* Before anything, we have a few notes:
*
* Positive direction is clockwise from the perspective opposite to the motor axially.
*
* We are using REV 41 and GoBilda 5203 for the launcher because we are out of resources.
*
* Latest measurement gives 12150 ticks per 23.8125 in, for our tiles are not perfect.
*
* */

@Autonomous(name = "BLUE")
public class _AutonomousBlue_ extends LinearOpMode{

    public static class Intake{
        private final DcMotorEx GoBildaLauncher, REVLauncher;
        private final DcMotor GoBilda3_Intake;
        private final DcMotor REV_Loader;
        public double TPR = 28;

        public Intake(HardwareMap hardwareMap){
            //Launcher will help push the artefacts downwards so they will not get stuck.
            GoBildaLauncher = hardwareMap.get(DcMotorEx.class, "GoBildaLauncher");
            GoBildaLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            GoBildaLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            GoBildaLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncher = hardwareMap.get(DcMotorEx.class, "REVLauncher");
            REVLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Basic intake with rubber bands for friction.
            GoBilda3_Intake = hardwareMap.get(DcMotor.class, "GoBildaIntake");
            GoBilda3_Intake.setDirection(DcMotorSimple.Direction.REVERSE);
            GoBilda3_Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REV_Loader = hardwareMap.get(DcMotor.class, "REVLoader");
            REV_Loader.setDirection(DcMotorSimple.Direction.REVERSE);
            REV_Loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public class Take_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBilda3_Intake.setPower(1);
                REV_Loader.setPower(0.35);
                GoBildaLauncher.setVelocity((-1100.0 / 60.0) * TPR);
                REVLauncher.setVelocity((-1100.0 / 60.0) * TPR);
                return false;
            }
        }
        public Action takeArtefact(){
            return new Take_Artefact();
        }

        public class Discard_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBilda3_Intake.setPower(-1);
                REV_Loader.setPower(-0.79);
                return false;
            }
        }
        public Action discardArtefact(){
            return new Discard_Artefact();
        }

        public class Keep_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBilda3_Intake.setPower(0);
                REV_Loader.setPower(0);
                return false;
            }
        }
        public Action keepArtefact(){
            return new Keep_Artefact();
        }
    }

    public static class Turret{

        private final DcMotorEx GoBildaLauncher, REVLauncher;
        private final DcMotor REVLoader, GoBildaIntake;
        Servo Aimer;
        double LaunchSpeed = 2500;

        /*
        * Launch speed and servo aiming was determined quasi-empirically.
        *
        *   POINT       LAUNCHER        SERVO       DISTANCE TO TARGET
        *   (+00;+00)   2830            0.5         101.823
        *   (-12;-12)   2650            0.25        084.853
        *   (-24;-24)   2500            0           067.882
        *
        * Position for blue target: (-72;-72).
        *
        * Launcher function: y = 0.0521006 x^2 + 0.881022 x + 2200.11667
        *
        * Aimer function: y = 0.0147314 x - 1
        *
        * Each field tile is 24in by 24in.
        *
        * Two launch locations: (-24;-24) and (+60;-12)
        *
        * Launch lines: x = |y| + 48 and x = -|y|.
        *
        * Two launch locations:
        *
        * POINT         DISTANCE        LAUNCHER        AIMER
        * (-24;-24)     067.882         2500            0
        * (+60;-12)     144.996         3425            1
        * */

        public double TPR = 28;
        PIDFCoefficients PIDF = new PIDFCoefficients(300, 0.5, 0, 19);

        private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        public Turret (HardwareMap hardwareMap){
            GoBildaLauncher = hardwareMap.get(DcMotorEx.class, "GoBildaLauncher");
            GoBildaLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            GoBildaLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            GoBildaLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            GoBildaLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
            REVLauncher = hardwareMap.get(DcMotorEx.class, "REVLauncher");
            REVLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

            GoBildaIntake = hardwareMap.get(DcMotor.class, "GoBildaIntake");
            GoBildaIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            GoBildaIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLoader = hardwareMap.get(DcMotor.class, "REVLoader");
            REVLoader.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            Aimer = hardwareMap.get(Servo.class, "ServoAimer");
            Aimer.setPosition(0);
        }

        /*
        * Both GoBilda and REV have their motors running on 28 ticks each revolution.
        * Desired speed in rotation per minute will be divided by 60 to get per second rotation.
        * Multiply that with 28 we have the desired angular speed.
        * */

        public class Spool_Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                Aimer.setPosition(0);
                GoBildaLauncher.setVelocity((LaunchSpeed / 60) * TPR);
                REVLauncher.setVelocity((LaunchSpeed / 60) * TPR);
                //Here a tolerance of 10 ticks will be allowed because we have had enough.
                return (GoBildaLauncher.getVelocity() + REVLauncher.getVelocity()) / 2 < (LaunchSpeed / 60) * TPR - 10;
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
                GoBildaIntake.setPower(0.78);
                REVLoader.setPower(1);
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
                GoBildaLauncher.setVelocity(0);
                REVLauncher.setVelocity(0);
                GoBildaLauncher.setPower(0);
                REVLauncher.setPower(0);
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
        Pose2d Location = new Pose2d(-61, -6, Math.toRadians(0));
        MecanumDrive Base = new MecanumDrive(hardwareMap, Location);
        Intake intake = new Intake(hardwareMap);
        Turret launcher = new Turret(hardwareMap);
        waitForStart();
        Action Path1 = Base.actionBuilder(Location)
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(225))
                .build();
        Action Path2 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(-8, -20), Math.toRadians(270))
                .build();
        Action Path3 = Base.actionBuilder(new Pose2d(-8, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(-8, -52))
                .build();
        Action Path4 = Base.actionBuilder(new Pose2d(-8, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(225))
                .build();
        Action Path5 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(12, -20), Math.toRadians(270))
                .build();
        Action Path6 = Base.actionBuilder(new Pose2d(12, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, -52))
                .build();
        Action Path7 = Base.actionBuilder(new Pose2d(12, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(225))
                .build();
        Action Path8 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(36, -20), Math.toRadians(270))
                .build();
        Action Path9 = Base.actionBuilder(new Pose2d(36, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(36, -52))
                .build();
        Action PathA = Base.actionBuilder(new Pose2d(36, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(216))
                .build();

        ParallelAction Spool1 = new ParallelAction(
                Path1,
                launcher.spoolUp()
        );
        ParallelAction Defer2 = new ParallelAction(
                Path2,
                intake.discardArtefact()
        );
        ParallelAction Refer3 = new ParallelAction(
                Path3,
                intake.takeArtefact()
        );
        ParallelAction Spool4 = new ParallelAction(
                Path4,
                launcher.spoolUp(),
                intake.keepArtefact()
        );
        ParallelAction Defer5 = new ParallelAction(
                Path5,
                intake.discardArtefact()
        );
        ParallelAction Refer6 = new ParallelAction(
                Path6,
                intake.takeArtefact()
        );
        ParallelAction Spool7 = new ParallelAction(
                Path7,
                launcher.spoolUp(),
                intake.keepArtefact()
        );
        ParallelAction Defer8 = new ParallelAction(
                Path8,
                intake.discardArtefact()
        );
        ParallelAction Refer9 = new ParallelAction(
                Path9,
                intake.takeArtefact()
        );
        ParallelAction SpoolA = new ParallelAction(
                PathA,
                launcher.spoolUp(),
                intake.keepArtefact()
        );

        SequentialAction Blue = new SequentialAction(
                Spool1,
                launcher.launchArtefact(),
                launcher.halt(),
                Defer2,
                Refer3,
                intake.keepArtefact(),
                Spool4,
                launcher.launchArtefact(),
                launcher.halt(),
                Defer5,
                Refer6,
                intake.keepArtefact(),
                Spool7,
                launcher.launchArtefact(),
                launcher.halt(),
                Defer8,
                Refer9,
                intake.keepArtefact(),
                SpoolA,
                launcher.launchArtefact(),
                launcher.halt()
        );

        Actions.runBlocking(Blue);
    }
}

//ENDING POSITION: (+60;-12); HEADING 216.
