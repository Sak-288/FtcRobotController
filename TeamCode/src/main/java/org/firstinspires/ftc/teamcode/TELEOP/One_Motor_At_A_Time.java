package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class One_Motor_At_A_Time extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx rightExpulsionMotor = hardwareMap.get(DcMotorEx.class, "REM");
        DcMotorEx leftExpulsionMotor = hardwareMap.get(DcMotorEx.class, "LEM");

        rightExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightExpulsionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftExpulsionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightExpulsionMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftExpulsionMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // CONSTANTS at initialization
        double BASE_TICKS_PER_REV_DC = 28;
        double BASE_TICKS_PER_REV_CORE = 288;
        double DT_MOTOR_RPM = 6000.0;
        double INTAKE_MOTOR_RPM = 125;
        double RAMP_MOTOR_RPM = 6000.0;
        double EXPULSION_MOTOR_RPM = 6000.0;
        double DT_GEARBOX_RATIO = 4.0 * 5.0;
        double INTAKE_GEARBOX_RATIO = 1.0;
        double RAMP_MOTOR_GEARBOX_RATIO = 3.0;
        double EXPULSION_MOTOR_GEARBOX_RATIO = 3.0;

        double RPM_to_RPS = 1 / 60.0;

        double DT_SPEED = 1.0;
        double Strafing_Correction = 1.12;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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
            double intakePow = gamepad2.right_trigger;
            double rampPow = gamepad2.left_trigger;

            double expulsionPow = gamepad2.a ? -1.0 : 0.0;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x - rx) / denominator;
            double backLeftPower = (-y + x + rx) / denominator;
            double frontRightPower = (-y + x - rx) / denominator;
            double backRightPower = (y + x + rx) / denominator;

            rightExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * EXPULSION_MOTOR_GEARBOX_RATIO * expulsionPow);
            leftExpulsionMotor.setVelocity(EXPULSION_MOTOR_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * EXPULSION_MOTOR_GEARBOX_RATIO * expulsionPow);
        }
    }
}

