package org.firstinspires.ftc.teamcode.AUTO_with_PP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class BottomBlueAuto extends OpMode {
    // Initializing PP stuff instances
    public Follower follower;
    public Timer pathTimer, opModeTimer;

    // Now PP States
    private String pathState;

    // ----- POSES -----
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose parkPose = new Pose(105, 33, Math.toRadians(180));
    private final Pose scorePose = new Pose(52, 108, Math.toRadians(135));
    private final Pose thirdLineStart = new Pose(42, 36, Math.toRadians(180));
    private final Pose thirdLineEnd = new Pose(12, 36, Math.toRadians(180));
    private final Pose secondLineStart = new Pose(42, 60, Math.toRadians(180));
    private final Pose secondLineEnd = new Pose(12, 60, Math.toRadians(180));
    private final Pose firstLineStart = new Pose(42, 84, Math.toRadians(180));
    private final Pose firstLineEnd = new Pose(12, 84, Math.toRadians(180));
    private final Pose loadingZone = new Pose(132, 16, Math.toRadians(180)); // Not using that one for now

    // Now for the path chains :: They need to be built before the AUTO actually starts
    private PathChain shootFirst, pickUpFirst, shootSecond, pickUpSecond, shootThird, pickUpThird, shootFourth, goHome;

    public void buildPaths() {
        // Shoot the first preloaded batch
        shootFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(72, 48), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .build();

        // Then go pick up the closest batch
        pickUpFirst = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, firstLineStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstLineStart.getHeading())
                .setBrakingStart(1) // FOR NOW, everywhere is the same, aka DEFAULT. This may be changed during testing.
                .setBrakingStrength(1) // DEFAULT TOO
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .addPath(new BezierLine(firstLineStart, firstLineEnd))
                .setLinearHeadingInterpolation(firstLineStart.getHeading(), firstLineEnd.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .build();

        // Then shoot that batch you just picked up
        shootSecond = follower.pathBuilder()
                .addPath(new BezierCurve(firstLineEnd, new Pose(firstLineStart.getX(), firstLineStart.getY()), scorePose))
                .setLinearHeadingInterpolation(firstLineEnd.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .build();

        // Then get that second closest batch
        pickUpSecond = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(firstLineStart.getX(), firstLineStart.getY()), secondLineStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLineStart.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .addPath(new BezierLine(secondLineStart, secondLineEnd))
                .setLinearHeadingInterpolation(secondLineStart.getHeading(), secondLineEnd.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .build();

        // Then go shoot that second closest batch
        shootThird = follower.pathBuilder()
                .addPath(new BezierCurve(secondLineEnd, new Pose(secondLineStart.getX(), secondLineStart.getY()), scorePose))
                .setLinearHeadingInterpolation(secondLineEnd.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .build();

        // Then go pickup that last far-out batch
        pickUpThird = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(secondLineStart.getX(), secondLineStart.getY()), thirdLineStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLineStart.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .addPath(new BezierLine(thirdLineStart, thirdLineEnd))
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .setLinearHeadingInterpolation(thirdLineStart.getHeading(), thirdLineEnd.getHeading())
                .build();

        // Then go shoot that last batch
        shootFourth = follower.pathBuilder()
                .addPath(new BezierCurve(thirdLineEnd, new Pose(thirdLineStart.getX(), thirdLineStart.getY()), scorePose))
                .setLinearHeadingInterpolation(thirdLineEnd.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .build();

        // Then go park for extra Ranking Point, ig ?
        goHome = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(72, 48), parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .setTimeoutConstraint(100) // SAME, but TIME IS WHACK AF.
                .setTValueConstraint(0.99) // SAME, but this is HEAVILY PREFERRED.
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "shootFirst":
                if (!follower.isBusy()) {
                    follower.followPath(shootFirst, true);
                    setPathState("pickUpFirst");
                }
                break;
            case "pickUpFirst":
                if(!follower.isBusy()) {
                    follower.followPath(pickUpFirst,true);
                    setPathState("shootSecond");
                }
                break;
            case "shootSecond":
                if(!follower.isBusy()) {
                    follower.followPath(shootSecond,true);
                    setPathState("pickUpSecond");
                }
                break;
            case "pickUpSecond":
                if(!follower.isBusy()) {
                    follower.followPath(pickUpSecond,true);
                    setPathState("shootThird");
                }
                break;
            case "shootThird":
                if(!follower.isBusy()) {
                    follower.followPath(shootThird,true);
                    setPathState("pickUpThird");
                }
                break;
            case "pickUpThird":
                if(!follower.isBusy()) {
                    follower.followPath(pickUpThird,true);
                    setPathState("shootFourth");
                }
                break;
            case "shootFourth":
                if(!follower.isBusy()) {
                    follower.followPath(shootFourth, true);
                    setPathState("goHome");
                }
                break;
            case "goHome":
                if(!follower.isBusy()) {
                    follower.followPath(goHome, true);
                    setPathState("default");
                }
                break;
            case "default":
                break;
        }
    }

    // This changes the states of the paths and actions. It will also reset the timers of the individual switches
    public void setPathState(String givenPathState) {
        pathState = givenPathState;
        pathTimer.resetTimer(); // Who is ever going to use the timer ? So random.
    }

    // Weird Initialization button on Driver Hub
    @Override
    public void init() {
        // Setting up the Timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        // Instantiating Follower with the Constants found during Tuning... Good Luck...
        follower = Constants.createFollower(hardwareMap);
        // PreAmogusing the paths for better runtime
        buildPaths();
        follower.setStartingPose(startPose);
    }

    // This supposedly (awaiting test) runs once after the "PLAY" button has been pressed by the drivers
    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState("shootFirst");
    }

    // The really important stuff -- RUNTIME
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Current Path State : ", pathState);
        telemetry.addData("Current Time : ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Constraints : ", follower.getConstraints());

        telemetry.addData("X : ", follower.getPose().getX());
        telemetry.addData("Y : ", follower.getPose().getY());
        telemetry.addData("Heading : ", follower.getPose().getHeading());
        telemetry.update();
    }
}


