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

@TeleOp(name = "T_RED")
public class _TeleOpRed_ extends LinearOpMode{

    DcMotorEx REVLauncherR, REVLauncherL;

    DcMotor GoBildaLoader, REVLoader;

    Servo Aimer, Pusher;

    CRServo TSL, TSR;

    public double P = 1000000;

    public double F = 3100;

    PIDFCoefficients PIDF = new PIDFCoefficients(P, 0, 0, F);

    @Override
    public void runOpMode() throws InterruptedException{

        MecanumDrive base = new MecanumDrive(hardwareMap, new Pose2d(0 ,24 ,0));

        /*
         * Positive rotation direction is clockwise from the perspective opposite the motor.
         * */

        REVLauncherR = hardwareMap.get(DcMotorEx.class, "REVR");
        REVLauncherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        REVLauncherR.setDirection(DcMotorSimple.Direction.FORWARD);
        REVLauncherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REVLauncherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        REVLauncherL = hardwareMap.get(DcMotorEx.class, "REVL");
        REVLauncherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        REVLauncherL.setDirection(DcMotorSimple.Direction.REVERSE);
        REVLauncherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REVLauncherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        /*
         * All intake motors go counterclockwise.
         * */

        GoBildaLoader = hardwareMap.get(DcMotor.class, "GoBildaIntake");
        GoBildaLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GoBildaLoader.setDirection(DcMotorSimple.Direction.REVERSE);

        REVLoader = hardwareMap.get(DcMotor.class, "REVTransfer");
        REVLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        REVLoader.setDirection(DcMotorSimple.Direction.REVERSE);

        TSL = hardwareMap.get(CRServo.class, "ServoL");
        TSR = hardwareMap.get(CRServo.class, "ServoR");

        Aimer = hardwareMap.get(Servo.class, "Hood");

        Pusher = hardwareMap.get(Servo.class, "Pusher");

        base.updatePoseEstimate();

        double LaunchVelocity;

        double Distance_Target;

        double ServoAngle;

        final double LinearScalar = 1;

        final double AngularScalar = 1;

        double[] Target = {-72, 72};

        double Offset = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() & !isStopRequested()){

            //ROBOT CENTRIC, NOT PLAYER CENTRIC.

            Vector2d Motion = new Vector2d(
                    -gamepad1.left_stick_y * LinearScalar,
                    -gamepad1.left_stick_x * LinearScalar
            );

            base.setDrivePowers(
                    new PoseVelocity2d(
                            Motion,
                            -gamepad1.right_stick_x * AngularScalar
                    )
            );

            base.updatePoseEstimate();

            telemetry.addData("x", base.localizer.getPose().position.x);
            telemetry.addData("y", base.localizer.getPose().position.y);
            telemetry.addData("h", base.localizer.getPose().heading.toDouble());

            //TARGET LOCATION IS AT (-72;+72).

            Distance_Target = Math.sqrt(Math.pow(base.localizer.getPose().position.x - Target[0], 2)
                    + Math.pow(base.localizer.getPose().position.y - Target[1], 2));

            /*
             * Position for red target: (-72;+72).
             * Launcher function: y = 12.96368 x + 1476.66352.
             * Aimer function: y = 0.0294629 x - 2.
             * */

            LaunchVelocity = 12.96368 * Distance_Target + 1476.66352 + Offset;

            ServoAngle = 0.0294629 * Distance_Target - 2;

            if (ServoAngle > 1) {
                ServoAngle = 1;
            } else if (ServoAngle < 0){
                ServoAngle = 0;
            }
            Aimer.setPosition(ServoAngle);
            TSL.setPower(0);
            TSR.setPower(0);

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
            //It's a miracle the code works.
            if (gamepad1.a){
                REVLauncherL.setVelocity((-1100.0 / 60.0) * 28);
                REVLauncherR.setVelocity((-1100.0 / 60.0) * 28);
            }
            else if (gamepad1.left_trigger != 0){
                REVLauncherL.setVelocity((LaunchVelocity / 60) * 28);
                REVLauncherR.setVelocity((LaunchVelocity / 60) * 28);
            } else {
                REVLauncherL.setVelocity(0);
                REVLauncherR.setVelocity(0);
            }
            if (gamepad1.right_trigger > 0.5){
                Pusher.setPosition(1);
            } else {
                Pusher.setPosition(0);
            }
            if (gamepad2.dpadDownWasPressed()){
                base = new MecanumDrive(hardwareMap, new Pose2d(0, 24, 0));
            }
            if (gamepad1.dpadUpWasPressed()){
                Offset += 10;
            } else if (gamepad1.dpadDownWasPressed()){
                Offset -= 10;
            }
            telemetry.addData("Offset", Offset);
            telemetry.update();
        }

    }

}
