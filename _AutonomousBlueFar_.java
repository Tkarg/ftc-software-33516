
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

@Autonomous(name = "A_BLUE_F")
public class _AutonomousBlueFar_ extends LinearOpMode{

    public static class ArtefactHandler{

        private final DcMotorEx REVLauncherR, REVLauncherL;
        private final DcMotor REVLoader, GoBildaIntake;
        Servo Aimer;
        double LaunchSpeed = 3350;

        /*
         * Turns out the ideal launch speed for AUTO is going to be 3350RPM at point (+60;-12) for blue.
         * */

        public double TPR = 28;

        public double IntakeRunning = 0;

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
            Aimer.setPosition(1);
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
            private boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                if (!init) {
                    timer.reset();
                    init = true;
                }
                Aimer.setPosition(1);
                REVLauncherR.setVelocity((LaunchSpeed / 60) * TPR);
                REVLauncherL.setVelocity((LaunchSpeed / 60) * TPR);
                //We need some real speed.
                return (REVLauncherR.getVelocity() + REVLauncherL.getVelocity()) / 2 < (LaunchSpeed / 60) * TPR;
                //A very rudimentary checking method for obstructions.
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
                //HDPE.
                if (timer.time(TimeUnit.MILLISECONDS) < 300 |
                        (timer.time(TimeUnit.MILLISECONDS) > 600 & timer.time(TimeUnit.MILLISECONDS) < 900) |
                        (timer.time(TimeUnit.MILLISECONDS) > 1200 & timer.time(TimeUnit.MILLISECONDS) < 1500)) {
                    GoBildaIntake.setPower(1);
                    REVLoader.setPower(1);
                } else {
                    GoBildaIntake.setPower(0);
                    REVLoader.setPower(0);
                }
                return timer.time(TimeUnit.MILLISECONDS) < 1500;
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
        Pose2d Location = new Pose2d(60, -12, Math.toRadians(208));
        MecanumDrive Base = new MecanumDrive(hardwareMap, Location);
        ArtefactHandler artefactHandler = new ArtefactHandler(hardwareMap);
        waitForStart();
        //If there are two strategies this is two of them.
        Action Wait = Base.actionBuilder(Location)
                .waitSeconds(20)
                .build();
        Action Move = Base.actionBuilder(Location)
                .strafeToLinearHeading(new Vector2d(60, -30), Math.toRadians(180))
                .build();
        SequentialAction Far = new SequentialAction(
                Wait,
                artefactHandler.spoolUp(),
                artefactHandler.launchArtefact(),
                Move
        );
        Actions.runBlocking(Far);
    }
}

//ENDING POSITION: (+00;-24); HEADING 0.
