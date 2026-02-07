package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class One_Motor_At_A_Time extends LinearOpMode {

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx rightExpulsionMotor = hardwareMap.get(DcMotorEx.class, "REM");
        DcMotorEx leftExpulsionMotor = hardwareMap.get(DcMotorEx.class, "LEM");

        rightExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightExpulsionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftExpulsionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightExpulsionMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftExpulsionMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // CONSTANTS at initialization
        double BASE_TICKS_PER_REV_DC = 28;
        double MAX_EXPULSION_MOTOR_RPM = 6000.0;
        double EXPULSION_MOTOR_GEARBOX_RATIO = 1.0;

        double RPM_to_RPS = 1 / 60.0;

        double wantedRPM = 3000.0;

        boolean lastA = false;
        boolean lastB = false;
        boolean lastX = false;
        boolean lastY = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            if(gamepad1.a && !lastA){
                wantedRPM += 100;
            }
            if(gamepad1.b && !lastB){
                wantedRPM -=100;
            }
            if(gamepad1.x && !lastX){
                wantedRPM += 10;
            }
            if(gamepad1.y && !lastY){
                wantedRPM -= 10;
            }

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            telemetry.addLine(String.format("Expulsion RPM : " + wantedRPM));
            telemetry.update();

            rightExpulsionMotor.setVelocity(wantedRPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * EXPULSION_MOTOR_GEARBOX_RATIO * 1);
            leftExpulsionMotor.setVelocity(wantedRPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * EXPULSION_MOTOR_GEARBOX_RATIO * 1);
        }
    }
}

