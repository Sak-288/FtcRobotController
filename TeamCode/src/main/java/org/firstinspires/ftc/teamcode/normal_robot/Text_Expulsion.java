package org.firstinspires.ftc.teamcode.normal_robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Text_Expulsion extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftMotor = hardwareMap.dcMotor.get("leftMotor");
        DcMotor rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double expulsion_power = 0.0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean boolean_power = gamepad1.a;

            if (boolean_power){
                expulsion_power = 1.0;
            }
            else {
                expulsion_power = 0.0;
            }

            leftMotor.setPower(expulsion_power);
            rightMotor.setPower(expulsion_power);
        }
    }
}

