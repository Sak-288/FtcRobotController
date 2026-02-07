package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
        double BASE_TICKS_PER_REV_DC = 28;
        double BASE_TICKS_PER_REV_CORE = 288;

        double DT_MOTOR_RPM = 6000.0;
        double INTAKE_MOTOR_RPM = 6000.0;
        double RAMP_MOTOR_RPM = 6000.0;
        double EXPULSION_MOTOR_RPM = 6000.0;

        double DT_GEARBOX_RATIO = 4.0 * 5.0;
        double INTAKE_GEARBOX_RATIO = 1.0;
        double RAMP_MOTOR_GEARBOX_RATIO = 1.0;
        double EXPULSION_MOTOR_GEARBOX_RATIO = 1.0;

        double RPM_to_RPS = 1 / 60.0;

        double DT_SPEED = 1.0;
        double Strafing_Correction = 1.1;

        // This is to change expulsion motors speed
        double jjk_modulo = 0; // You'll change this eventually
        double modulation = 0.5;
        boolean lastA = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Gets the speed you wanna go at
            if (gamepad1.x) {
                DT_SPEED = 0.25;
            }
            if (gamepad1.a) {
                DT_SPEED = 0.50;
            }
            if (gamepad1.b) {
                DT_SPEED = 0.75;
            }
            if (gamepad1.y) {
                DT_SPEED = 1.00;
            }

            // Drivetrain Controller Controls
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * Strafing_Correction;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x - rx) / denominator);
            double backLeftPower = ((-y + x + rx) / denominator);
            double frontRightPower = ((-y + x - rx) / denominator);
            double backRightPower = ((y + x + rx) / denominator);

            // Mechanisms Controller Controls
            double intakePow = gamepad2.y ? 1.0 : 0.0;
            if (gamepad2.a && !lastA) {
                jjk_modulo += 1;
                if (jjk_modulo % 2 == 0) {
                    modulation = 0.5;
                } else {
                    modulation = 1.0;
                }
            }
            if (jjk_modulo % 2 == 0) {

            }
            double expulsionPow = gamepad2.x ? modulation * 1.0 : 0.0;
            double rampPow = gamepad2.a ? 1.0 : 0.0;

            // Ugly expulsion motor speed modulation for now, if it works (hopefully), it works.
            lastA = gamepad2.a;

            // DRIVETRAIN MOTORS
            frontLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO * frontLeftPower * DT_SPEED);
            backLeftMotor.setVelocity(DT_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO * backLeftPower * DT_SPEED);
            frontRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO * frontRightPower * DT_SPEED);
            backRightMotor.setVelocity(DT_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO* backRightPower * DT_SPEED);

            // MECHANISM MOTORS
            IntakeMotor.setVelocity(INTAKE_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_CORE * INTAKE_GEARBOX_RATIO * intakePow);
            RampMotor.setVelocity(RAMP_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * RAMP_MOTOR_GEARBOX_RATIO * rampPow);
            leftExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * EXPULSION_MOTOR_GEARBOX_RATIO * expulsionPow);
            rightExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * EXPULSION_MOTOR_GEARBOX_RATIO * expulsionPow);
        }
    }
}

