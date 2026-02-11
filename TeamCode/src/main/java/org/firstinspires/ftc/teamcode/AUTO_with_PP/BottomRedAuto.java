package org.firstinspires.ftc.teamcode.AUTO_with_PP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Bottom Blue Auto - Final", group = "Autonomous")
public class BottomRedAuto extends OpMode {

    public Follower follower;
    public Timer pathTimer, opModeTimer;
    private String pathState;

    // ----- POSES -----
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(88, 108, Math.toRadians(45));
    private final Pose firstLineStart = new Pose(102, 84, Math.toRadians(0));
    private final Pose firstLineEnd = new Pose(128, 84, Math.toRadians(0));
    private final Pose secondLineStart = new Pose(102, 60, Math.toRadians(0));
    private final Pose secondLineEnd = new Pose(128, 60, Math.toRadians(0));
    private final Pose thirdLineStart = new Pose(102, 36, Math.toRadians(0));
    private final Pose thirdLineEnd = new Pose(128, 36, Math.toRadians(0));
    private final Pose parkPose = new Pose(39, 34, Math.toRadians(90));

    // Path Chains
    private PathChain shootFirst, pickUpFirst, shootSecond, pickUpSecond, shootThird, goHome;

    // Motors
    private DcMotorEx IntakeMotor, RampMotor, leftExpulsionMotor, rightExpulsionMotor;

    // Constants
    private final double BASE_TICKS_PER_REV_DC = 28;
    private final double BASE_TICKS_PER_REV_CORE = 288;
    private final double RPM_to_RPS = 1 / 60.0;

    private double nearShootingRPM = 3000.0;
    private double intakingRPM = 125.0;

    double SHOOTING_TIME = 3.0; // Reverted as requested
    private double INTAKING_TIME = 0.8;

    public void buildPaths() {
        shootFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(72, 48), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100)
                .setTValueConstraint(0.99)
                .build();

        pickUpFirst = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, firstLineStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstLineStart.getHeading())
                .addPath(new BezierLine(firstLineStart, firstLineEnd))
                .setLinearHeadingInterpolation(firstLineStart.getHeading(), firstLineEnd.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100)
                .setTValueConstraint(0.99)
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(new BezierCurve(firstLineEnd, new Pose(firstLineStart.getX(), firstLineStart.getY()), scorePose))
                .setLinearHeadingInterpolation(firstLineEnd.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100)
                .setTValueConstraint(0.99)
                .build();

        pickUpSecond = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(firstLineStart.getX(), firstLineStart.getY()), secondLineStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLineStart.getHeading())
                .addPath(new BezierLine(secondLineStart, secondLineEnd))
                .setLinearHeadingInterpolation(secondLineStart.getHeading(), secondLineEnd.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100)
                .setTValueConstraint(0.99)
                .build();

        shootThird = follower.pathBuilder()
                .addPath(new BezierCurve(secondLineEnd, new Pose(secondLineStart.getX(), secondLineStart.getY()), scorePose))
                .setLinearHeadingInterpolation(secondLineEnd.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100)
                .setTValueConstraint(0.99)
                .build();

        goHome = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(72, 48), parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100)
                .setTValueConstraint(0.99)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "START":
                follower.followPath(shootFirst, true);
                setPathState("DRIVE_TO_SCORE_1");
                break;

            case "DRIVE_TO_SCORE_1":
                if (!follower.isBusy()) {
                    setPathState("SHOOT_1");
                }
                break;

            case "SHOOT_1":
                setShooterPower(nearShootingRPM);
                if (pathTimer.getElapsedTimeSeconds() > SHOOTING_TIME) {
                    setShooterPower(0);
                    follower.followPath(pickUpFirst, true);
                    setPathState("DRIVE_TO_PICKUP_1");
                }
                break;

            case "DRIVE_TO_PICKUP_1":
                setIntakePower(intakingRPM);
                if (!follower.isBusy()) {
                    setPathState("INTAKE_WAIT_1");
                }
                break;

            case "INTAKE_WAIT_1":
                if (pathTimer.getElapsedTimeSeconds() > INTAKING_TIME) {
                    setIntakePower(0);
                    follower.followPath(shootSecond, true);
                    setPathState("DRIVE_TO_SCORE_2");
                }
                break;

            case "DRIVE_TO_SCORE_2":
                if (!follower.isBusy()) {
                    setPathState("SHOOT_2");
                }
                break;

            case "SHOOT_2":
                setShooterPower(nearShootingRPM);
                if (pathTimer.getElapsedTimeSeconds() > SHOOTING_TIME) {
                    setShooterPower(0);
                    follower.followPath(pickUpSecond, true);
                    setPathState("DRIVE_TO_PICKUP_2");
                }
                break;

            case "DRIVE_TO_PICKUP_2":
                setIntakePower(intakingRPM);
                if (!follower.isBusy()) {
                    setPathState("INTAKE_WAIT_2");
                }
                break;

            case "INTAKE_WAIT_2":
                if (pathTimer.getElapsedTimeSeconds() > INTAKING_TIME) {
                    setIntakePower(0);
                    follower.followPath(shootThird, true);
                    setPathState("DRIVE_TO_SCORE_3");
                }
                break;

            case "DRIVE_TO_SCORE_3":
                if (!follower.isBusy()) {
                    setPathState("SHOOT_3");
                }
                break;

            case "SHOOT_3":
                setShooterPower(nearShootingRPM);
                if (pathTimer.getElapsedTimeSeconds() > SHOOTING_TIME) {
                    setShooterPower(0);
                    follower.followPath(goHome, true);
                    setPathState("DRIVE_TO_PARK");
                }
                break;

            case "DRIVE_TO_PARK":
                if (!follower.isBusy()) {
                    setPathState("END");
                }
                break;

            case "END":
                break;
        }
    }

    public void setPathState(String state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IM");
        RampMotor = hardwareMap.get(DcMotorEx.class, "RM");
        leftExpulsionMotor = hardwareMap.get(DcMotorEx.class, "LEM");
        rightExpulsionMotor = hardwareMap.get(DcMotorEx.class, "REM");

        configureMotor(IntakeMotor, DcMotorEx.Direction.FORWARD);
        configureMotor(RampMotor, DcMotorEx.Direction.FORWARD);
        configureMotor(leftExpulsionMotor, DcMotorEx.Direction.FORWARD);
        configureMotor(rightExpulsionMotor, DcMotorEx.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    private void configureMotor(DcMotorEx motor, DcMotorEx.Direction dir) {
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(dir);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState("START");
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    public void setShooterPower(double rpm) {
        double velocity = rpm * RPM_to_RPS * BASE_TICKS_PER_REV_DC;
        double ramp_vel = 6000.0 * RPM_to_RPS * 28.0;
        RampMotor.setVelocity(ramp_vel);
        leftExpulsionMotor.setVelocity(velocity);
        rightExpulsionMotor.setVelocity(velocity);
    }

    public void setIntakePower(double rpm) {
        double velocity = rpm * RPM_to_RPS * BASE_TICKS_PER_REV_CORE;
        IntakeMotor.setVelocity(velocity);
    }
}