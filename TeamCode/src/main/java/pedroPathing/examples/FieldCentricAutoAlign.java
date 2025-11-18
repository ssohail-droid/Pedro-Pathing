package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "FieldCentricAutoAlign", group = "Examples")
public class FieldCentricAutoAlign extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(39, 128, Math.toRadians(270));
    private final Pose targetPose = new Pose(22, 122, Math.toRadians(311)); // Target point near wall

    // Distance sensors
    private DistanceSensor frontSensor;
    private DistanceSensor rightSensor;

    // Button and state tracking
    private boolean xPressed = false;
    private boolean navigating = false;
    private boolean aligning = false;
    private boolean arrived = false;

    // Tuning constants (adjust for your robot!)
    private static final double TARGET_FRONT_DIST = 2;   // inches from wall
    private static final double TARGET_RIGHT_DIST = 15;   // inches from side wall
    private static final double ALIGN_TOLERANCE = 1.0;     // inches
    private static final double ALIGN_SPEED = 0.1;         // low speed for fine adjustment

    private static final double POSITION_TOLERANCE = 2.0;
    private static final double HEADING_TOLERANCE = Math.toRadians(5);

    @Override
    public void init() {
        // Init follower and pose
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Init sensors
        frontSensor = hardwareMap.get(DistanceSensor.class, "front_distance");
        rightSensor = hardwareMap.get(DistanceSensor.class, "right_distance");

        telemetry.addLine("Initialized. Press X to auto-align.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();  // Start in manual drive
    }

    @Override
    public void loop() {
        // === Handle X to start alignment sequence ===
        if (gamepad1.x && !xPressed && !navigating && !aligning) {
            xPressed = true;
            startAutoNavigation();
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        // === Auto navigation to target ===
        if (navigating && !arrived) {
            follower.update();
            if (hasArrivedAtTarget()) {
                navigating = false;
                arrived = true;

                follower.breakFollowing();
                follower.setPose(targetPose);
                follower.startTeleopDrive();

                aligning = true; // Now start fine sensor-based alignment
            }
        }

        // === Fine alignment using distance sensors ===
        if (aligning) {
            boolean alignedFront = false;
            boolean alignedRight = false;

            double frontDist = frontSensor.getDistance(DistanceUnit.INCH);
            double rightDist = rightSensor.getDistance(DistanceUnit.INCH);

            double yPower = 0;
            double xPower = 0;

            if (Math.abs(frontDist - TARGET_FRONT_DIST) > ALIGN_TOLERANCE) {
                yPower = (frontDist > TARGET_FRONT_DIST) ? ALIGN_SPEED : -ALIGN_SPEED;
            } else {
                alignedFront = true;
            }

            if (Math.abs(rightDist - TARGET_RIGHT_DIST) > ALIGN_TOLERANCE) {
                xPower = (rightDist > TARGET_RIGHT_DIST) ? ALIGN_SPEED : -ALIGN_SPEED;
            } else {
                alignedRight = true;
            }

            // Apply small alignment movement
            if (!(alignedFront && alignedRight)) {
                follower.setTeleOpMovementVectors(xPower, yPower, 0, false);
                follower.update();
            } else {
                // Alignment complete
                aligning = false;
                arrived = false;
                telemetry.addLine(">>> Auto-alignment complete. <<<");
            }
        }

        // === Manual drive if not auto aligning ===
        if (!navigating && !aligning) {
            follower.setTeleOpMovementVectors(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // === Telemetry ===
        telemetry.addLine(navigating ? "Navigating to target..." :
                aligning ? "Auto-aligning with sensors..." :
                        "Manual drive mode");

        telemetry.addLine("=== Pose ===");
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("=== Sensors ===");
        telemetry.addData("Front Distance", "%.2f in", frontSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance", "%.2f in", rightSensor.getDistance(DistanceUnit.INCH));

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

    private boolean hasArrivedAtTarget() {
        Pose current = follower.getPose();
        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distanceError = Math.hypot(dx, dy);

        double headingError = normalizeAngle(targetPose.getHeading() - current.getHeading());

        return distanceError < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
