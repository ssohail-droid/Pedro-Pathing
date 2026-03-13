package pedroPathing.State;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "TELE RED FRONT(OLD)TETE", group = "Main")
@Config
public class DrveTest extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(95.99999999999999, 126.53915776241358, Math.toRadians(0));

    public static Pose TARGET_A = new Pose(107.3, 106, Math.toRadians(42));
    public static Pose TARGET_B = new Pose(105.331136738056, 108.4, Math.toRadians(42));
    public static Pose TARGET_C = new Pose(93.5, 14.38, Math.toRadians(71));

    public static double POS_TOL = 2.0;
    public static double HEAD_TOL_DEG = 0.01;

    public static double SLOW_MULT = 0.35;
    public static double TRIGGER_MIN_MULT = 0.25;

    private Pose activeTarget = null;
    private boolean navigating = false;
    private boolean slowMode = false;
    private boolean lastSlowToggle = false;
    private boolean lastXDrive = false;
    private boolean lastYDrive = false;
    private boolean lastBDrive = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        /* ===== SLOW MODE TOGGLE ===== */
        boolean slowTogglePressed = gamepad1.a && !lastSlowToggle;
        if (slowTogglePressed) slowMode = !slowMode;
        lastSlowToggle = gamepad1.a;

        /* ===== TARGET SELECT ===== */
        boolean xPressed = gamepad1.x && !lastXDrive;
        boolean yPressed = gamepad1.y && !lastYDrive;
        boolean bPressed = gamepad1.b && !lastBDrive;

        if (!navigating) {
            if (xPressed) {
                activeTarget = TARGET_A;
                navigating = true;
            } else if (yPressed) {
                activeTarget = TARGET_B;
                navigating = true;
            } else if (bPressed) {
                activeTarget = TARGET_C;
                navigating = true;
            }

            if (navigating) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(activeTarget)
                                ))
                                .setLinearHeadingInterpolation(
                                        follower.getPose().getHeading(),
                                        activeTarget.getHeading()
                                )
                                .build()
                );
            }
        }

        lastXDrive = gamepad1.x;
        lastYDrive = gamepad1.y;
        lastBDrive = gamepad1.b;

        /* ===== CANCEL NAVIGATION ===== */
        if ((gamepad1.left_bumper || gamepad1.right_bumper) && navigating) {
            navigating = false;
            activeTarget = null;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        /* ===== DRIVE ===== */
        if (navigating) {
            follower.update();
            if (arrived()) {
                navigating = false;
                activeTarget = null;
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
        } else {
            double trigger = gamepad1.right_trigger;
            double triggerMult = 1.0 - trigger * (1.0 - TRIGGER_MIN_MULT);
            double finalMult = slowMode ? Math.min(SLOW_MULT, triggerMult) : triggerMult;

            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y * finalMult,
                    -gamepad1.left_stick_x * finalMult,
                    -gamepad1.right_stick_x * finalMult,
                    false
            );
            follower.update();
        }

        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Navigating", navigating);
        telemetry.update();
    }

    private boolean arrived() {
        if (activeTarget == null) return false;

        Pose p = follower.getPose();
        double dx = activeTarget.getX() - p.getX();
        double dy = activeTarget.getY() - p.getY();
        double dist = Math.hypot(dx, dy);

        double headingErr = Math.toDegrees(
                Math.atan2(
                        Math.sin(p.getHeading() - activeTarget.getHeading()),
                        Math.cos(p.getHeading() - activeTarget.getHeading())
                )
        );

        return dist < POS_TOL && Math.abs(headingErr) < HEAD_TOL_DEG;
    }
}
