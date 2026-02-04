package pedroPathing.Jan10.TELEOP.DASAUTO;

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

// plz disable later B4 Comp
@Autonomous(name = "Red Auto Joint Pathing", group = "Examples")
public class RedAutoJointPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    /* ================= ENUM STATE MACHINE ================= */

    private enum AutoState {
        MOVE_TO_SHOOT,
        MOVE_ROW1,
        PICKUP_ROW1,
        OPEN_GATE_CURVE,
        RETURN_SHOOT1,

        MOVE_ROW2,
        PICKUP_ROW2,
        RETURN_SHOOT2,

        LEAVE,
        DONE
    }

    private AutoState state = AutoState.MOVE_TO_SHOOT;

    /* ================= POSES ================= */

    private final Pose startPos = new Pose(111, 135, Math.toRadians(0));
    private final Pose shootPos = new Pose(105.331136738056, 108.4, Math.toRadians(42));

    private final Pose intakeRowOnePos = new Pose(95, 93, Math.toRadians(180));
    private final Pose intakePickUpRowOnePos = new Pose(125, 93, Math.toRadians(180));

    private final Pose openGate = new Pose(127.02337075207845, 90, Math.toRadians(180));
    private final Pose openGateControlPoint = new Pose(100, 78.03255813953488);

    private final Pose intakeRowTwoPos = new Pose(100, 70, Math.toRadians(180));
    private final Pose intakePickUpRowTwoPos = new Pose(131, 67, Math.toRadians(180));

    private final Pose leave = new Pose(95.99999999999999, 126.53915776241358, Math.toRadians(0));

    /* ================= PATHS ================= */

    private PathChain moveToShoot, moveToIntakeRowOne, moveToPickUpRowOne;
    private PathChain moveToOpenGate, moveRoundTwoShoot;

    private PathChain moveIntakeRowTwo, movePickUpRowTwo, moveShootRowTwo;
    private PathChain moveLeave;

    public void buildPaths() {

        moveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

        moveToIntakeRowOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowOnePos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowOnePos.getHeading())
                .build();

        moveToPickUpRowOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowOnePos), new Point(intakePickUpRowOnePos)))
                .setLinearHeadingInterpolation(intakeRowOnePos.getHeading(), intakePickUpRowOnePos.getHeading())
                .build();

        moveToOpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowOnePos),
                        new Point(openGateControlPoint.getX(), openGateControlPoint.getY()),
                        new Point(openGate)
                ))
                .setLinearHeadingInterpolation(
                        intakePickUpRowOnePos.getHeading(),
                        openGate.getHeading()
                )
                .build();

        moveRoundTwoShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(openGate), new Point(shootPos)))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootPos.getHeading())
                .build();

        moveIntakeRowTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowTwoPos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowTwoPos.getHeading())
                .build();

        movePickUpRowTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowTwoPos), new Point(intakePickUpRowTwoPos)))
                .setLinearHeadingInterpolation(intakeRowTwoPos.getHeading(), intakePickUpRowTwoPos.getHeading())
                .build();

        Point moveSevenControl = new Point(100.929, 58.8972);

        moveShootRowTwo = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowTwoPos),
                        moveSevenControl,
                        new Point(shootPos)
                ))
                .setLinearHeadingInterpolation(
                        intakePickUpRowTwoPos.getHeading(),
                        shootPos.getHeading()
                )
                .build();

        moveLeave = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(leave)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), leave.getHeading())
                .build();
    }

    /* ================= ENUM FSM UPDATE ================= */

    public void autonomousPathUpdate() {

        switch (state) {

            case MOVE_TO_SHOOT:
                follower.followPath(moveToShoot);
                state = AutoState.MOVE_ROW1;
                follower.setMaxPower(0.6);
                break;

            case MOVE_ROW1:
                if (!follower.isBusy()) {
                    follower.followPath(moveToIntakeRowOne);
                    state = AutoState.PICKUP_ROW1;
                    follower.setMaxPower(0.6);
                }
                break;

            case PICKUP_ROW1:
                if (!follower.isBusy()) {
                    follower.followPath(moveToPickUpRowOne);
                    state = AutoState.OPEN_GATE_CURVE;
                }
                break;

            case OPEN_GATE_CURVE:
                if (!follower.isBusy()) {
                    follower.followPath(moveToOpenGate);
                    state = AutoState.RETURN_SHOOT1;
                }
                break;

            case RETURN_SHOOT1:
                if (!follower.isBusy()) {
                    follower.followPath(moveRoundTwoShoot);
                    state = AutoState.MOVE_ROW2;
                }
                break;

            case MOVE_ROW2:
                if (!follower.isBusy()) {
                    follower.followPath(moveIntakeRowTwo);
                    state = AutoState.PICKUP_ROW2;
                }
                break;

            case PICKUP_ROW2:
                if (!follower.isBusy()) {
                    follower.followPath(movePickUpRowTwo);
                    state = AutoState.RETURN_SHOOT2;
                }
                break;

            case RETURN_SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(moveShootRowTwo);
                    state = AutoState.LEAVE;
                }
                break;

            case LEAVE:
                if (!follower.isBusy()) {
                    follower.followPath(moveLeave);
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                // Auto finished
                break;
        }
    }

    /* ================= INIT ================= */

    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);

        buildPaths();
    }

    @Override
    public void start() {
        state = AutoState.MOVE_TO_SHOOT;
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}