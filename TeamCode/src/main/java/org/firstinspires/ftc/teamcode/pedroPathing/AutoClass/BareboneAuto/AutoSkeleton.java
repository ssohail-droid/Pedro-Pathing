package org.firstinspires.ftc.teamcode.pedroPathing.AutoClass.BareboneAuto;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


//@Autonomous(name = "AutoSkeleton(WillDoNothing)", group = "Examples")
public class AutoSkeleton extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Place your unit tests, pose-related variables, PathChain configurations,
    // motor variables, and servo variables in this section.
    private final Pose startPos = new Pose(9, 72, Math.toRadians(0));//just an example

    public void buildPaths() {
        // This is where you build and configure the path using the PathBuilder.
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