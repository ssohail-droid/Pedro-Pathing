package pedroPathing.AutoClass.EnumeratedState;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "EnumeratedStateAuto(V2.0)", group = "Examples")
public class EnumeratedStateAutoTwo extends OpMode {

    // ===== CONSTANTS =====
    private static final Pose START_POSE = new Pose(9, 72, Math.toRadians(180));
    private static final Pose SUBMERSIBLE_POSE = new Pose(33.5, 72, Math.toRadians(180));
    private static final double MOVEMENT_SPEED = 0.5;
    private static final double PRECISION_SPEED = 0.3;
    private static final double ACTION_TIMEOUT = 5.0;
    private static final double SETTLING_TIME = 0.5;

    // ===== STATE ENUM =====
    public enum State {
        INITIALIZE,
        MOVING_TO_SUBMERSIBLE,
        AT_SUBMERSIBLE,
        RETURNING_HOME,
        COMPLETE,
        ERROR
    }

    // ===== MAIN VARIABLES =====
    private Follower follower;
    private Timer pathTimer, opmodeTimer, stateTimer;

    // State management
    private State currentState = State.INITIALIZE;
    private boolean pathStarted = false;

    // Paths
    private PathChain moveToSubmersible;
    private PathChain moveBackHome;

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        stateTimer = new Timer();
        opmodeTimer.resetTimer();

        // Setup Pedro Pathing
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // Build paths
        moveToSubmersible = buildStraightPath(START_POSE, SUBMERSIBLE_POSE);
        moveBackHome = buildStraightPath(SUBMERSIBLE_POSE, START_POSE);
    }

    @Override
    public void loop() {
        try {
            follower.update();
            updateStateMachine();
            updateTelemetry();
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            currentState = State.ERROR;
        }
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addData("Start Position", START_POSE.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setState(State.INITIALIZE);
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    // ===== STATE MACHINE =====
    private void updateStateMachine() {
        switch (currentState) {
            case INITIALIZE:
                // Wait a brief moment before starting
                if (stateTimer.getElapsedTimeSeconds() > 0.1) {
                    setState(State.MOVING_TO_SUBMERSIBLE);
                }
                break;

            case MOVING_TO_SUBMERSIBLE:
                if (!pathStarted) {
                    startPath(moveToSubmersible, MOVEMENT_SPEED);
                }

                if (isPathComplete()) {
                    setState(State.AT_SUBMERSIBLE);
                }

                // Timeout protection
                if (pathTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setState(State.ERROR);
                }
                break;

            case AT_SUBMERSIBLE:
                // Perform actions at submersible (scoring, etc.)
                // For now, just wait briefly then return home
                if (stateTimer.getElapsedTimeSeconds() > SETTLING_TIME) {
                    setState(State.RETURNING_HOME);
                }
                break;

            case RETURNING_HOME:
                if (!pathStarted) {
                    startPath(moveBackHome, MOVEMENT_SPEED);
                }

                if (isPathComplete()) {
                    setState(State.COMPLETE);
                }

                // Timeout protection
                if (pathTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setState(State.ERROR);
                }
                break;

            case COMPLETE:
                // Autonomous complete - do nothing
                break;

            case ERROR:
                // Stop all movement
                follower.breakFollowing();
                break;

            default:
                setState(State.ERROR);
                break;
        }
    }

    // ===== HELPER METHODS =====
    private void startPath(PathChain path, double maxPower) {
        follower.setMaxPower(maxPower);
        follower.followPath(path);
        pathStarted = true;
        pathTimer.resetTimer();
    }

    private boolean isPathComplete() {
        return pathStarted && !follower.isBusy();
    }

    private void setState(State newState) {
        currentState = newState;
        pathStarted = false;
        stateTimer.resetTimer();
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Path Started", pathStarted);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X Position", "%.2f", follower.getPose().getX());
        telemetry.addData("Y Position", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("OpMode Time", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("State Time", "%.1f", stateTimer.getElapsedTimeSeconds());

        if (pathStarted) {
            telemetry.addData("Path Time", "%.1f", pathTimer.getElapsedTimeSeconds());
        }
    }

    // ===== PATH BUILDING METHODS =====
    private PathChain buildStraightPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(start), new Point(end)))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    private PathChain buildCurvedPath(Pose start, Pose end, Point controlPoint1, Point controlPoint2) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(start), controlPoint1, controlPoint2, new Point(end)))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    // Alternative: Build curved path with single control point
    private PathChain buildSimpleCurvedPath(Pose start, Pose end, Point controlPoint) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(start), controlPoint, new Point(end)))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    private PathChain buildConstantHeadingPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(start), new Point(end)))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();
    }
}