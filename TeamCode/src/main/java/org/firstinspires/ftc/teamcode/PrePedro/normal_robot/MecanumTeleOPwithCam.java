package org.firstinspires.ftc.teamcode.PrePedro.normal_robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PrePedro.normal_robot.mecahnisms_outside.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
public class MecanumTeleOPwithCam extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(); // Instantiate new webcam
        aprilTagWebcam.init(hardwareMap, telemetry);

        // Declare our motors
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FLM");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "BLM");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "FRM");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "BRM");
        DcMotorEx IntakeMotor = hardwareMap.get(DcMotorEx.class, "IM");
        DcMotorEx RampMotor = hardwareMap.get(DcMotorEx.class, "RM");
        DcMotorEx leftExpulsionMotor = hardwareMap.get(DcMotorEx.class, "LEM");
        DcMotorEx rightExpulsionMotor = hardwareMap.get(DcMotorEx.class, "REM");

        // Set the mode
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RampMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set Zero Power Behaviour
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RampMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftExpulsionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightExpulsionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Setting motor directions
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE); // Reverse the right ones, or the left ones, depending on robot orientation
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        RampMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftExpulsionMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightExpulsionMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // CONSTANTS at initialization
        double DT_MOTOR_RPM = 6000.0 / 20.0;
        double INTAKE_MOTOR_RPM = 125;
        double RAMP_MOTOR_RPM = 6000.0 / 3.0;
        double EXPULSION_MOTOR_RPM = 6000.0 / 3.0;

        double RPM_to_TICKS = 1 / 60.0;

        double DT_SPEED = 1.0;
        double Strafing_Correction = 1.20;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            aprilTagWebcam.update();
            AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23); // TEST with PPG

            List<Double> idTagCoords = aprilTagWebcam.displayDetectionTelemetry(id23);

            // Following or Directing
            boolean Following = gamepad2.a;

            if (Following) {
                if (idTagCoords == null) {
                    // Gets the speed you wanna go at
                    boolean quarterSpeed = gamepad1.x;
                    boolean halfSpeed = gamepad1.a;
                    boolean threeSpeed = gamepad1.b;
                    boolean fullSpeed = gamepad1.y;

                    if (quarterSpeed) {
                        DT_SPEED = 0.25;
                    }
                    if (halfSpeed) {
                        DT_SPEED = 0.50;
                    }
                    if (threeSpeed) {
                        DT_SPEED = 0.75;
                    }
                    if (fullSpeed) {
                        DT_SPEED = 1.00;
                    }

                    // Drivetrain Controller Controls
                    double y = -gamepad1.left_stick_y * DT_SPEED;
                    double x = gamepad1.left_stick_x * Strafing_Correction * DT_SPEED;
                    double rx = -gamepad1.right_stick_x * DT_SPEED;

                    // Mechanisms Controller Controls
                    double intakePow = -gamepad2.right_trigger;
                    double rampPow = gamepad2.left_trigger;
                    double expulsionPow = gamepad2.a ? 1.0 : 0.0;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x - rx) / denominator;
                    double backLeftPower = (-y + x + rx) / denominator;
                    double frontRightPower = (-y + x - rx) / denominator;
                    double backRightPower = (y + x + rx) / denominator;

                    frontLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * frontLeftPower);
                    backLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * backLeftPower);
                    frontRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * frontRightPower);
                    backRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * backRightPower);
                    IntakeMotor.setVelocity(INTAKE_MOTOR_RPM * RPM_to_TICKS * intakePow);
                    RampMotor.setVelocity(RAMP_MOTOR_RPM * RPM_to_TICKS * rampPow);
                    leftExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_TICKS * expulsionPow);
                    rightExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_TICKS * expulsionPow);
                }
                else {
                    // Gets the speed you wanna go at
                    boolean quarterSpeed = gamepad1.x;
                    boolean halfSpeed = gamepad1.a;
                    boolean threeSpeed = gamepad1.b;
                    boolean fullSpeed = gamepad1.y;

                    if (quarterSpeed) {
                        DT_SPEED = 0.25;
                    }
                    if (halfSpeed) {
                        DT_SPEED = 0.50;
                    }
                    if (threeSpeed) {
                        DT_SPEED = 0.75;
                    }
                    if (fullSpeed) {
                        DT_SPEED = 1.00;
                    }

                    double DAMP_FACTOR = 0.05;

                    double y = aprilTagWebcam.displayDetectionTelemetry(id23).get(1) * DT_SPEED * DAMP_FACTOR * 2;
                    double x = -aprilTagWebcam.displayDetectionTelemetry(id23).get(0) * DT_SPEED * DAMP_FACTOR;
                    double rx = aprilTagWebcam.displayDetectionTelemetry(id23).get(2) * DT_SPEED * DAMP_FACTOR;

                    // Mechanisms Controller Controls
                    double intakePow = -gamepad2.right_trigger;
                    double rampPow = gamepad2.left_trigger;

                    double expulsionPow = gamepad2.a ? 1.0 : 0.0;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x - rx) / denominator;
                    double backLeftPower = (-y + x + rx) / denominator;
                    double frontRightPower = (-y + x - rx) / denominator;
                    double backRightPower = (y + x + rx) / denominator;

                    frontLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * frontLeftPower);
                    backLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * backLeftPower);
                    frontRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * frontRightPower);
                    backRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * backRightPower);
                    IntakeMotor.setVelocity(INTAKE_MOTOR_RPM * RPM_to_TICKS * intakePow);
                    RampMotor.setVelocity(RAMP_MOTOR_RPM * RPM_to_TICKS * rampPow);
                    leftExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_TICKS * expulsionPow);
                    rightExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_TICKS * expulsionPow);
                }
            } else {
                // Gets the speed you wanna go at
                boolean quarterSpeed = gamepad1.x;
                boolean halfSpeed = gamepad1.a;
                boolean threeSpeed = gamepad1.b;
                boolean fullSpeed = gamepad1.y;

                if (quarterSpeed) {
                    DT_SPEED = 0.25;
                }
                if (halfSpeed) {
                    DT_SPEED = 0.50;
                }
                if (threeSpeed) {
                    DT_SPEED = 0.75;
                }
                if (fullSpeed) {
                    DT_SPEED = 1.00;
                }

                // Drivetrain Controller Controls
                double y = -gamepad1.left_stick_y * DT_SPEED;
                double x = gamepad1.left_stick_x * Strafing_Correction * DT_SPEED;
                double rx = -gamepad1.right_stick_x * DT_SPEED;

                // Mechanisms Controller Controls
                double intakePow = -gamepad2.right_trigger;
                double rampPow = gamepad2.left_trigger;

                double expulsionPow = gamepad2.a ? 1.0 : 0.0;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x - rx) / denominator;
                double backLeftPower = (-y + x + rx) / denominator;
                double frontRightPower = (-y + x - rx) / denominator;
                double backRightPower = (y + x + rx) / denominator;

                frontLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * frontLeftPower);
                backLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * backLeftPower);
                frontRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * frontRightPower);
                backRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_TICKS * backRightPower);
                IntakeMotor.setVelocity(INTAKE_MOTOR_RPM * RPM_to_TICKS * intakePow);
                RampMotor.setVelocity(RAMP_MOTOR_RPM * RPM_to_TICKS * rampPow);
                leftExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_TICKS * expulsionPow);
                rightExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_TICKS * expulsionPow);
            }
        }
    }
}