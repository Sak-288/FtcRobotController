package org.firstinspires.ftc.teamcode.AUTO_with_PP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BottomBlueAuto", group = "Test")
public class NormalAuto extends LinearOpMode {

    // --- HARDWARE DECLARATIONS (Made Global) ---
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx IntakeMotor, RampMotor, leftExpulsionMotor, rightExpulsionMotor;
    private final ElapsedTime runtime = new ElapsedTime();

    // --- CONSTANTS ---
    static final double TICKS_PER_REV = 560.0;
    static final double WHEEL_DIAMETER_INCHES = 2.96;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double STRAFE_CORRECTION = 1.15;
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

        // 2. SET DIRECTIONS
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        RampMotor.setDirection(DcMotorEx.Direction.FORWARD);
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

        // THE MAIN SCRIPT
        double DT_SPEED = 0.80;

        drive(24, 0, 0, DT_SPEED);
        start_intake();
        drive(0, 24, 0, DT_SPEED);
        end_intake();
        outtake(3.0);

        telemetry.addData("Status", "AUTO Complete");
        telemetry.update();
    }

    // --- HELPER METHODS ---

    private void drive(double axial, double lateral, double turn, double power) {
        double adjLateral = lateral * STRAFE_CORRECTION;

        int flT = frontLeftMotor.getCurrentPosition() + (int)((axial + adjLateral + turn) * TICKS_PER_INCH);
        int blT = backLeftMotor.getCurrentPosition() + (int)((axial - adjLateral + turn) * TICKS_PER_INCH);
        int frT = frontRightMotor.getCurrentPosition() + (int)((axial - adjLateral - turn) * TICKS_PER_INCH);
        int brT = backRightMotor.getCurrentPosition() + (int)((axial + adjLateral - turn) * TICKS_PER_INCH);

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
        // HD Hex is 6000 RPM. (6000/60) * 28 ticks = 2800 ticks/sec
        leftExpulsionMotor.setVelocity(2800);
        rightExpulsionMotor.setVelocity(2800);
        RampMotor.setVelocity(1400); // Ramp half speed

        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addLine("Expelling...");
            telemetry.update();
        }
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