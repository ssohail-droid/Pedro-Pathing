package pedroPathing.AutoClass.EnumeratedState.refactored;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

public class AutoStateMachine {
    private final Follower follower;
    private final Timer pathTimer = new Timer();

    private final Pose startPos = new Pose(9, 72, Math.toRadians(180));
    private final Pose submersiblePos = new Pose(33.5, 72, Math.toRadians(180));

    private PathChain moveToSubmersible;
    private PathChain moveToSubmersible2;

    private State state = State.STATE_START;
    private boolean pathStarted = false;

    public AutoStateMachine(Follower follower) {
        this.follower = follower;
        this.follower.setStartingPose(startPos);
        buildPaths();
    }

    private void buildPaths() {
        moveToSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(submersiblePos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), submersiblePos.getHeading())
                .build();

        moveToSubmersible2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePos), new Point(startPos)))
                .setLinearHeadingInterpolation(submersiblePos.getHeading(), startPos.getHeading())
                .build();
    }

    public void update() {
        switch (state) {
            case STATE_START:
                if (!follower.isBusy()) {
                    setState(State.STATE_MOVE_TO_SUBMERSIBLE);
                }
                break;

            case STATE_MOVE_TO_SUBMERSIBLE:
                if (!pathStarted) {
                    follower.setMaxPower(0.5);
                    follower.followPath(moveToSubmersible);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setState(State.STATE_MOVE_TO_DEBUG);
                }
                break;

            case STATE_MOVE_TO_DEBUG:
                if (!pathStarted) {
                    follower.setMaxPower(0.5);
                    follower.followPath(moveToSubmersible2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setState(State.STATE_HOME);
                }
                break;

            case STATE_HOME:
                // Done. Add actions if needed.
                break;
        }
    }

    public void setState(State newState) {
        this.state = newState;
        this.pathStarted = false;
        this.pathTimer.resetTimer();
    }

    public State getState() {
        return state;
    }

    public Pose getPose() {
        return follower.getPose();
    }
}
