package pedroPathing.AutoClass.Dec6;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "FieldCentricRedAuto", group = "Examples")
public class FieldCentricRedAuto extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(39, 128, Math.toRadians(270));
    private final Pose targetPose = new Pose(22, 122, Math.toRadians(311));

    // Control flags
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean navigating = false;
    private boolean arrived = false;

    // Tolerances
    private static final double POSITION_TOLERANCE = 2.0;    // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(5); // radians

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addLine("Initialized. Press A to auto-drive.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();  // Start in manual control
    }

    @Override
    public void loop() {
        // === A to start auto navigation ===
        if (gamepad1.a && !aPressed && !navigating) {
            aPressed = true;
            startAutoNavigation();
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        // === B to cancel auto navigation ===
        if (gamepad1.b && !bPressed && navigating) {
            bPressed = true;
            cancelAutoNavigation();
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        // === Navigation logic ===
        if (navigating && !arrived) {
            follower.update();

            if (hasArrivedAtTarget()) {
                navigating = false;
                arrived = true;

                follower.breakFollowing();
                follower.setPose(targetPose);
                follower.startTeleopDrive();
            }
        }

        // === Manual drive (always active if not navigating) ===
        if (!navigating) {
            follower.setTeleOpMovementVectors(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // === Telemetry ===
        telemetry.addLine(navigating ? "Navigating to Target..." :
                (arrived ? "Arrived — Manual Drive Active" : "Manual Drive Mode"));
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void startAutoNavigation() {
        navigating = true;
        arrived = false;

        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(follower.getPose()),
                                new Point(targetPose)
                        ))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                        .build()
        );
    }

    private void cancelAutoNavigation() {
        navigating = false;
        arrived = false;

        follower.breakFollowing();
        follower.startTeleopDrive();
    }

    private boolean hasArrivedAtTarget() {
        Pose current = follower.getPose();

        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distanceError = Math.hypot(dx, dy);

        double headingError = normalizeAngle(targetPose.getHeading() - current.getHeading());

        return distanceError < POSITION_TOLERANCE &&
                Math.abs(headingError) < HEADING_TOLERANCE;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
