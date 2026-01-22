package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "TELEOP")
public class _TeleOpBlue_ extends LinearOpMode{

    DcMotorEx GoBildaLauncher, REVLauncher;

    DcMotor GoBildaLoader, REVLoader;

    CRServo C1, C2;

    Servo Aimer;

    PIDFCoefficients PIDF = new PIDFCoefficients(550, 1.5, 0, 19);

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
     * */

    @Override
    public void runOpMode() throws InterruptedException{

        MecanumDrive base = new MecanumDrive(hardwareMap, new Pose2d(60 ,-12 ,0));


        /*
        * Positive rotation direction is clockwise from the perspective opposite the motor.
        * */

        GoBildaLauncher = hardwareMap.get(DcMotorEx.class, "GoBildaLauncher");
        GoBildaLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        GoBildaLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        GoBildaLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GoBildaLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        REVLauncher = hardwareMap.get(DcMotorEx.class, "REVLauncher");
        REVLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        REVLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        REVLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REVLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        /*
        * All intake motors go counterclockwise.
        * */

        GoBildaLoader = hardwareMap.get(DcMotor.class, "GoBildaIntake");
        GoBildaLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GoBildaLoader.setDirection(DcMotorSimple.Direction.REVERSE);

        REVLoader = hardwareMap.get(DcMotor.class, "REVLoader");
        REVLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        REVLoader.setDirection(DcMotorSimple.Direction.REVERSE);

        C1 = hardwareMap.get(CRServo.class, "CRServoL");
        C2 = hardwareMap.get(CRServo.class, "CRServoR");

        Aimer = hardwareMap.get(Servo.class, "ServoAimer");

        base.updatePoseEstimate();

        double LaunchVelocity = 0;

        double Distance_Target = 0;

        double ServoAngle = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() & !isStopRequested()){

            Vector2d Motion = new Vector2d(
                    -gamepad1.left_stick_y * Math.cos(Math.toRadians(base.localizer.getPose().heading.toDouble()))
                    - gamepad1.left_stick_y * Math.sin(Math.toRadians(base.localizer.getPose().heading.toDouble())),
                    -gamepad1.left_stick_x * (-Math.sin(Math.toRadians(base.localizer.getPose().heading.toDouble())))
                    - gamepad1.left_stick_x * Math.cos(Math.toRadians(base.localizer.getPose().heading.toDouble()))
            );

            base.setDrivePowers(
                    new PoseVelocity2d(
                            Motion,
                            - gamepad1.right_stick_x
                    )
            );

            base.updatePoseEstimate();

            telemetry.addData("x", base.localizer.getPose().position.x);
            telemetry.addData("y", base.localizer.getPose().position.y);
            telemetry.addData("h", base.localizer.getPose().heading);
            telemetry.addData("REVLaunchSpeed", 28 * REVLauncher.getVelocity() / 60);
            telemetry.addData("GBDLaunchSpeed", 28 * GoBildaLauncher.getVelocity() / 60);

            //TARGET LOCATION IS AT (-72;-72).

            Distance_Target = Math.sqrt(Math.pow(base.localizer.getPose().position.x + 72, 2)
                    + Math.pow(base.localizer.getPose().position.y + 72, 2));

            LaunchVelocity = 0.0521006 * Math.pow(Distance_Target, 2) + 0.881022 * Distance_Target + 2200.11667;

            ServoAngle = 0.0147314 * Distance_Target - 1;

            if (ServoAngle > 1) {
                ServoAngle = 1;
            } else if (ServoAngle < 0){
                ServoAngle = 0;
            }
            Aimer.setPosition(ServoAngle);

            if (gamepad1.right_trigger != 0) {
                GoBildaLoader.setPower(1);
            } else {
                GoBildaLoader.setPower(0);
            }
            if (gamepad1.right_bumper){
                REVLoader.setPower(1);
            } else if (gamepad1.x){
                REVLoader.setPower(-1);
            } else {
                REVLoader.setPower(0);
            }
            if (gamepad1.a){
                REVLauncher.setVelocity((-LaunchVelocity / 60.0) * 28);
                GoBildaLauncher.setVelocity((-LaunchVelocity / 60.0) * 28);
            }
            if (gamepad1.left_trigger != 0){
                REVLauncher.setVelocity((LaunchVelocity / 60.0) * 28);
                GoBildaLauncher.setVelocity((LaunchVelocity / 60.0) * 28);
            } else {
                REVLauncher.setVelocity(0);
                GoBildaLauncher.setVelocity(0);
            }
            if (gamepad2.left_stick_x < 0){
                C1.setPower(-0.3);
                C2.setPower(-0.3);
            } else if (gamepad2.left_stick_x > 0){
                C1.setPower(0.3);
                C2.setPower(0.3);
            } else {
                C1.setPower(0);
                C2.setPower(0);
            }
            if (gamepad2.right_trigger != 0){
                Aimer.setPosition(0);
            } else if (gamepad2.right_bumper){
                Aimer.setPosition(1);
            }
            telemetry.update();
        }

    }

}
