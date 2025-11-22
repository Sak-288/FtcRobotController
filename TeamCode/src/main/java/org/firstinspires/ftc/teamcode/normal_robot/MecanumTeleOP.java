package org.firstinspires.ftc.teamcode.normal_robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mecahnisms_outside.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class MecanumTeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        CRServo OuttakeRampFirst = hardwareMap.get(CRServo.class, "OuttakeRampFirst");
        CRServo OuttakeRampSecond = hardwareMap.get(CRServo.class, "OuttakeRampSecond");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(); // Instantiate new webcam
        aprilTagWebcam.init(hardwareMap, telemetry);

        double SPEED = 0.0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            aprilTagWebcam.update();
            AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23); // TEST with PPG
            aprilTagWebcam.displayDetectionTelemetry(id23);

            // Gets the speed you wanna go at
            boolean quarterSpeed = gamepad1.x;
            boolean halfSpeed = gamepad1.a;
            boolean threeSpeed = gamepad1.b;
            boolean fullSpeed = gamepad1.y;


            if (quarterSpeed){
                SPEED = 0.25;
            } else if (halfSpeed){
                SPEED = 0.50;
            } else if (threeSpeed){
                SPEED = 0.75;
            } else if (fullSpeed) {
                SPEED = 1.00;
            }

            double y = -gamepad1.left_stick_y * SPEED; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1 * SPEED; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x * SPEED;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x - rx) / denominator;
            double backLeftPower = (-y + x + rx) / denominator;
            double frontRightPower = (-y + x - rx) / denominator;
            double backRightPower = (y + x + rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad2.x) {
                OuttakeRampFirst.setPower(1);
            }
            else {
                OuttakeRampFirst.setPower(0);
            }
            if (gamepad2.b) {
                OuttakeRampSecond.setPower(1);
            }
            else {
                OuttakeRampSecond.setPower(0);
            }
        }
    }
}

