package pedroPathing.AutoClass.EnumeratedState.refactored;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


//@Autonomous(name = "AutoTestOne()", group = "Examples")
public class AutoTestOne extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Place your unit tests, pose-related variables, PathChain configurations,
    // motor variables, and servo variables in this section.
    private final Pose startPos = new Pose(9, 9, Math.toRadians(0));//just an example
    private final Pose endPos = new Pose(9, 9, Math.toRadians(360));

    private PathChain firstPath;
    public void buildPaths() {
        // This is where you build and configure the path using the PathBuilder.

        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(endPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), endPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {}
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        buildPaths();

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
    @Override
    public void init_loop() {}


    @Override
    public void start() {

        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void stop() {
    }


}