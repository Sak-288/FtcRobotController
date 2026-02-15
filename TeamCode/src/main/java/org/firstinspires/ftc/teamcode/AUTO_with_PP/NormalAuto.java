package org.firstinspires.ftc.teamcode.AUTO_with_PP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BottomBlueAuto", group = "Test")
public class NormalAuto extends LinearOpMode {

    // --- HARDWARE DECLARATIONS (Made Global) ---
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, leftExpulsionMotor, rightExpulsionMotor;
    private DcMotorEx IntakeMotor, RampMotor;
    private Servo myServo;
    private final ElapsedTime runtime = new ElapsedTime();

    // --- CONSTANTS ---
    static final double TICKS_PER_REV = 560.0;
    static final double WHEEL_DIAMETER_INCHES = 2.96;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI) * 48/53;
    static final double STRAFE_CORRECTION = 1.00;
    static final double TURN_CORRECTION = 1.30;
    static final double TRACK_WIDTH_INCHES = 15.75;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. HARDWARE MAPPING
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FLM");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BLM");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FRM");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BRM");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IM");
        RampMotor = hardwareMap.get(DcMotorEx.class, "RM");
        leftExpulsionMotor = hardwareMap.get(DcMotorEx.class, "LEM");
        rightExpulsionMotor = hardwareMap.get(DcMotorEx.class, "REM");
        myServo = hardwareMap.get(Servo.class, "OuttakeServo");

        // 2. SET DIRECTIONS
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        RampMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftExpulsionMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightExpulsionMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // 3. BRAKE BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // 4. INITIAL MODES
        stopAndResetEncoders();

        // Mechanisms use RUN_USING_ENCODER for velocity control
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RampMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightExpulsionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        // THE MAIN SCRIPTw
        double DT_SPEED = 0.80;

        drive(24.0 * 3.2, 0, 0, DT_SPEED);
        double turnTicks = 135.0 / 360.0 * Math.PI * TRACK_WIDTH_INCHES;
        drive(0.0, 0.0, turnTicks, DT_SPEED / 2.0);
        outtake(3.0);
        turnTicks = -135.0 / 360.0 * Math.PI * TRACK_WIDTH_INCHES;
        drive(0.0, 0.0, turnTicks, DT_SPEED / 2.0);
        drive(24.0 * -0.75, 0.0, 0.0, DT_SPEED);
        turnTicks = -90.0 / 360.0 * Math.PI * TRACK_WIDTH_INCHES;
        drive(0.0, 0.0, turnTicks, DT_SPEED / 2.0);
        start_intake();
        drive(24.0 * 1.5, 0.0, 0.0, DT_SPEED / 4.0);
        end_intake();

        telemetry.addData("Status", "AUTO Complete");
        telemetry.update();
    }

    // --- HELPER METHODS ---

    private void drive(double axial, double lateral, double turn, double power) {
        double adjLateral = lateral * STRAFE_CORRECTION;
        double adjturn = turn * TURN_CORRECTION;

        // SWAPPED KINEMATICS
        int flT = frontLeftMotor.getCurrentPosition() + (int)((axial - adjLateral * 1.5 + adjturn) * TICKS_PER_INCH);
        int blT = backLeftMotor.getCurrentPosition() + (int)((axial + adjLateral + adjturn) * TICKS_PER_INCH);
        int frT = frontRightMotor.getCurrentPosition() + (int)((-axial - adjLateral + adjturn) * TICKS_PER_INCH);
        int brT = backRightMotor.getCurrentPosition() + (int)((-axial + adjLateral * 1.5 + adjturn) * TICKS_PER_INCH);

        frontLeftMotor.setTargetPosition(flT);
        backLeftMotor.setTargetPosition(blT);
        frontRightMotor.setTargetPosition(frT);
        backRightMotor.setTargetPosition(brT);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);

        while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy())) {
            telemetry.addLine("Moving Drivetrain...");
            telemetry.update();
        }

        stopDrivetrain();
    }

    private void start_intake() {
        IntakeMotor.setVelocity(600);
    }

    private void end_intake() {
        IntakeMotor.setVelocity(0.0);
    }
    private void outtake(double seconds) {
        runtime.reset();
        leftExpulsionMotor.setVelocity(1120);
        rightExpulsionMotor.setVelocity(1120);

        while (opModeIsActive() && runtime.seconds() < seconds) {
            myServo.setPosition(270.0/270);
            sleep(100);
            myServo.setPosition(160.0/270);
            RampMotor.setVelocity(900);
            sleep(400);
            RampMotor.setVelocity(0.0);

            telemetry.addLine("Flickering Outtake Servo...");
            telemetry.update();
        }

        // Ensure everything stops
        //
        leftExpulsionMotor.setVelocity(0);
        rightExpulsionMotor.setVelocity(0);
        RampMotor.setVelocity(0);
    }

    private void stopDrivetrain() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        // Reset to RUN_USING_ENCODER so isBusy() clears correctly
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAndResetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}