package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo; // Note the CRServo class

@TeleOp(name = "CR Servo Control")
public class servo extends LinearOpMode {

    private CRServo intakeServo;

    @Override
    public void runOpMode() {
        // Initialize as a CRServo
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        waitForStart();

        while (opModeIsActive()) {
            // 1. Joystick Control (Standard for Intakes)
            // We use -gamepad1.left_stick_y because pushing up is usually negative
            double power = -gamepad1.left_stick_y;

            // 2. Button Overrides
            if (gamepad1.right_bumper) {
                power = 1.0;  // Full speed intake
            } else if (gamepad1.left_bumper) {
                power = -1.0; // Full speed outtake
            } else if (gamepad1.x) {
                power = 0;    // Emergency stop
            }

            intakeServo.setPower(power);

            telemetry.addData("Servo Power", power);
            telemetry.update();
        }
    }
}