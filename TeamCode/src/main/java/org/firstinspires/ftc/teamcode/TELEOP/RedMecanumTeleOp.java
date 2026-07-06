package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class RedMecanumTeleOp extends LinearOpMode {
    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private TurretMechanismTutorial turret = new TurretMechanismTutorial();
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        // Camera instantation
        aprilTagWebcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap);

        // Declare our motors
        // DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FLM");
        //DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "BLM");
        //DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "FRM");
        //DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "BRM");

        //DcMotorEx firstIntakeMotor = hardwareMap.get(DcMotorEx.class, "FIM");
        //DcMotorEx secondIntakeMotor = hardwareMap.get(DcMotorEx.class, "SIM");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "SM");
        DcMotorEx rotativeMotor = hardwareMap.get(DcMotorEx.class, "RM");

        CRServo angleMotor = hardwareMap.get(CRServo.class, "AM");

        // Set the mode
        //frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //firstIntakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //secondIntakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotativeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set Zero Power Behaviour
        //frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //firstIntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //secondIntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rotativeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Setting motor directions
        //frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //firstIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //secondIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rotativeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // CONSTANTS at initialization
        double BASE_TICKS_PER_REV_DC = 28;
        double BASE_TICKS_PER_REV_CORE = 288;

        double DT_GEARBOX_RATIO = 4.0 * 5.0;
        double INTAKE_MOTOR_GEARBOX_RATIO = 1.0;
        double ROTATIVE_MOTOR_GEARBOX_RATIO = 1.0;
        double SHOOTER_MOTOR_GEARBOX_RATIO = 1.0;

        double DT_MOTOR_TARGET_RPM = 6000.0 / DT_GEARBOX_RATIO;
        double INTAKE_MOTOR_TARGET_RPM = 125.0 / INTAKE_MOTOR_GEARBOX_RATIO;
        double ROTATIVE_MOTOR_TARGET_RPM = 6000.0 / ROTATIVE_MOTOR_GEARBOX_RATIO;
        double SHOOTER_MOTOR_TARGET_RPM = 6000.0 / SHOOTER_MOTOR_GEARBOX_RATIO;

        double RPM_to_RPS = 1 / 60.0;

        double DT_SPEED = 0.50;
        double Strafing_Correction = 1.1; // Test this out again.

        double angleVel = 0.00;
        double rotativePow = 0.00;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Camera Stuff
            aprilTagWebcam.update();
            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

            turret.update(id24);

            // for tuning on the fly :
            if (gamepad1.bWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (gamepad1.dpadLeftWasPressed()){
                turret.setkP(turret.getkP() + stepSizes[stepIndex]);
            }
            if (gamepad1.dpadRightWasPressed()){
                turret.setkP(turret.getkP() - stepSizes[stepIndex]);
            }
            if (gamepad1.dpadUpWasPressed()){
                turret.setkD(turret.getkD() + stepSizes[stepIndex]);
            }
            if (gamepad1.dpadDownWasPressed()){
                turret.setkD(turret.getkD() - stepSizes[stepIndex]);
            }

            // Gets the speed you wanna go at
            //if (gamepad1.x) {
                //DT_SPEED = 0.25;
            //}
            //if (gamepad1.a) {
                //DT_SPEED = 0.50;
            //}
            //if (gamepad1.b) {
                //DT_SPEED = 0.75;
            //}
            //if (gamepad1.y) {
                //DT_SPEED = 1.00;
            //}

            // Drivetrain Controller Controls
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * Strafing_Correction;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y - x + rx) / denominator);
            double backLeftPower = ((y + x + rx) / denominator);
            double frontRightPower = ((-y - x + rx) / denominator);
            double backRightPower = ((-y + x + rx) / denominator);

            // Mechanisms Controller Controls
            double firstIntakePow = gamepad2.x ? 1.0 : 0.0;
            double secondIntakePow = gamepad2.y ? 1.0 : 0.0;
            double shooterPow = 0.7; // 0.7 * 6000 = 4200 RPM. Pretty damn fast. Constant.

            //if (gamepad2.a){
                //rotativePow = 0.4;
            //} else if (gamepad2.b){
                //rotativePow = -0.4;
            //} else {
                //rotativePow = 0.00;
            //}

            // This will replace right now, just for the sake of logic.

            // For testing only | will replace with experimental Regressi
            if (gamepad2.left_bumper){
                angleVel = -0.25;
            } else if (gamepad2.right_bumper){
                angleVel = 0.25;
            } else {
                angleVel = 0.00;
            }

            // DRIVETRAIN MOTORS
            //frontLeftMotor.setVelocity(DT_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO * frontLeftPower * DT_SPEED);
            //backLeftMotor.setVelocity(DT_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO * backLeftPower * DT_SPEED);
            //frontRightMotor.setVelocity(DT_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO * frontRightPower * DT_SPEED);
            //backRightMotor.setVelocity(DT_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * DT_GEARBOX_RATIO* backRightPower * DT_SPEED);
            angleMotor.setPower(angleVel);

            // MECHANISM MOTORS
            //firstIntakeMotor.setVelocity(INTAKE_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * INTAKE_MOTOR_GEARBOX_RATIO * firstIntakePow);
            //secondIntakeMotor.setVelocity(INTAKE_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * INTAKE_MOTOR_GEARBOX_RATIO * secondIntakePow);
            shooterMotor.setVelocity(SHOOTER_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_DC * SHOOTER_MOTOR_GEARBOX_RATIO * shooterPow);
            rotativeMotor.setVelocity(ROTATIVE_MOTOR_TARGET_RPM * RPM_to_RPS * BASE_TICKS_PER_REV_CORE * ROTATIVE_MOTOR_GEARBOX_RATIO * rotativePow);

            // USELESS TELEMETRY
            telemetry.update();
        }
    }
}