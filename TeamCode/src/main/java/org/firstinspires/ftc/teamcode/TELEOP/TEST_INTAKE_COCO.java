package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TEST_INTAKE_COCO extends LinearOpMode {

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx IntakeMotor = hardwareMap.get(DcMotorEx.class, "IM");

        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // CONSTANTS at initialization
        double BASE_TICKS_PER_REV_CORE = 288;
        double MAX_CORE_RPM = 125.0;
        double CORE_HEX_RATIO = 1.0;
        double RPM_to_RPS = 1 / 60.0;

        double pow = 0.0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            pow = gamepad1.a ? 1.0 : 0.0;
            IntakeMotor.setVelocity(MAX_CORE_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_CORE * CORE_HEX_RATIO);
        }
    }
}

