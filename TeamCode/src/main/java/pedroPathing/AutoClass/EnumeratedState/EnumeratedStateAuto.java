package pedroPathing.AutoClass.EnumeratedState;

import static pedroPathing.AutoClass.EnumeratedState.State.STATE_START;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "EnumeratedStateAuto(V1.0)", group = "Examples")
public class EnumeratedStateAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private State state = STATE_START;
    private boolean pathStarted = false;  // Flag to ensure followPath is called once per state

    // Pose variables - starting and target positions
    private final Pose startPos = new Pose(9, 72, Math.toRadians(180));
    private final Pose submersiblePos = new Pose(33.5, 72, Math.toRadians(180));

    private PathChain moveToSubmersible;
    private PathChain moveToSubmersible2;

    public void buildPaths() {
        moveToSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(submersiblePos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), submersiblePos.getHeading())
                .build();

        moveToSubmersible2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePos), new Point(startPos)))
                .setLinearHeadingInterpolation(submersiblePos.getHeading(), startPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (state) {
            case STATE_START:
                // Wait until follower is idle, then transition to move to submersible
                if (!follower.isBusy()) {
                    setPathState(State.STATE_MOVE_TO_SUBMERSIBLE);
                    pathStarted = false; // Reset flag for next state
                }
                break;

            case STATE_MOVE_TO_SUBMERSIBLE:
                // Start following path once
                if (!pathStarted) {
                    follower.setMaxPower(0.5);
                    follower.followPath(moveToSubmersible);
                    pathStarted = true;
                }
                // When path is complete, transition to next state
                if (pathStarted && !follower.isBusy()) {
                    setPathState(State.STATE_MOVE_TO_DEBUG);
                    pathStarted = false;
                }
                break;

            case STATE_MOVE_TO_DEBUG:
                // Start following return path once
                if (!pathStarted) {
                    follower.setMaxPower(0.5);
                    follower.followPath(moveToSubmersible2);
                    pathStarted = true;
                }
                // When path is complete, transition to HOME state
                if (pathStarted && !follower.isBusy()) {
                    setPathState(State.STATE_HOME);
                    pathStarted = false;
                }
                break;

            case STATE_HOME:
                // Final state — can add more logic here if needed
                break;

            default:
                // Safety fallback: do nothing or log unexpected state
                telemetry.addData("Error", "Unknown state: " + state);
                telemetry.update();
                break;
        }
    }

    public void setPathState(State pState) {
        state = pState;
        pathTimer.resetTimer();
        telemetry.addData("State changed to", state);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
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
        telemetry.addData("path state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(State.STATE_START);
        pathStarted = false;
    }

    @Override
    public void stop() {}
}
