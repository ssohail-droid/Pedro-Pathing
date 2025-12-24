package pedroPathing.tuners_tests.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * 24 Inch Square Test - Localization Accuracy Test
 *
 * This OpMode drives the robot in a 24" x 24" square pattern to test odometry accuracy.
 * The robot will:
 * 1. Drive forward 24 inches
 * 2. Strafe right 24 inches
 * 3. Drive backward 24 inches
 * 4. Strafe left 24 inches (back to start)
 *
 * At the end, it displays accuracy metrics showing how close the robot returned to the starting position.
 * This is useful for tuning your Pinpoint odometry and verifying localization accuracy.
 *
 * SETUP:
 * - Place robot in the center of a clear area with at least 30" of space in all directions
 * - Mark the starting position on the ground
 * - Run the OpMode
 * - Check if the robot returns to the marked starting position
 * - Review accuracy metrics on Driver Station and FTC Dashboard
 *
 * You can adjust the square size on FTC Dashboard (192.168.43.1:8080/dash)
 */
@Config
//@Autonomous(name = "Square Test 24 Inch", group = "Localization Tests")
public class SquareTest24Inch extends LinearOpMode {

    // Configurable square size (adjustable via FTC Dashboard)
    public static double SQUARE_SIZE = 24.0; // inches

    // Starting position (center of your test area)
    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING_DEG = 0.0; // Facing forward (0°)

    // Speed control (0.0 to 1.0)
    public static double DRIVE_SPEED = 0.6;

    // Wait time at each corner (seconds)
    public static double CORNER_WAIT_TIME = 0.5;

    private Follower follower;
    private MultipleTelemetry multiTelemetry;
    private Pose startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize PedroPathing constants
        Constants.setConstants(FConstants.class, LConstants.class);

        // Create starting pose
        startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));

        // Initialize follower
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Display configuration
        multiTelemetry.addLine("=== 24 Inch Square Test Ready ===");
        multiTelemetry.addLine();
        multiTelemetry.addData("Square Size", "%.1f inches", SQUARE_SIZE);
        multiTelemetry.addData("Start Position", "X=%.1f, Y=%.1f, H=%.1f°",
                START_X, START_Y, START_HEADING_DEG);
        multiTelemetry.addLine();
        multiTelemetry.addLine("📍 SETUP INSTRUCTIONS:");
        multiTelemetry.addLine("1. Place robot in center of clear area");
        multiTelemetry.addLine("2. Mark starting position on ground");
        multiTelemetry.addLine("3. Ensure " + (SQUARE_SIZE + 6) + "\" clearance in all directions");
        multiTelemetry.addLine();
        multiTelemetry.addLine("The robot will drive in a square pattern:");
        multiTelemetry.addLine("  Forward → Right → Backward → Left");
        multiTelemetry.addLine();
        multiTelemetry.addLine("Press PLAY to start test");
        multiTelemetry.addLine("💡 Adjust square size in FTC Dashboard");
        multiTelemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Re-read values in case they were changed in Dashboard
        startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        follower.setStartingPose(startPose);

        // Define the four corners of the square
        Pose corner1 = new Pose(START_X, START_Y + SQUARE_SIZE, Math.toRadians(START_HEADING_DEG)); // Forward
        Pose corner2 = new Pose(START_X + SQUARE_SIZE, START_Y + SQUARE_SIZE, Math.toRadians(START_HEADING_DEG)); // Right
        Pose corner3 = new Pose(START_X + SQUARE_SIZE, START_Y, Math.toRadians(START_HEADING_DEG)); // Backward
        Pose corner4 = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)); // Left (back to start)

        multiTelemetry.addLine("🚀 Starting Square Test...");
        multiTelemetry.update();
        sleep(500);

        // ========== SIDE 1: Drive Forward ==========
        driveTo(corner1, "Side 1/4: Forward " + SQUARE_SIZE + "\"", startPose);
        waitAtCorner(1);

        // ========== SIDE 2: Strafe Right ==========
        driveTo(corner2, "Side 2/4: Strafe Right " + SQUARE_SIZE + "\"", corner1);
        waitAtCorner(2);

        // ========== SIDE 3: Drive Backward ==========
        driveTo(corner3, "Side 3/4: Backward " + SQUARE_SIZE + "\"", corner2);
        waitAtCorner(3);

        // ========== SIDE 4: Strafe Left (Return to Start) ==========
        driveTo(corner4, "Side 4/4: Strafe Left " + SQUARE_SIZE + "\" (Returning to Start)", corner3);

        // ========== TEST COMPLETE - SHOW RESULTS ==========
        Pose finalPose = follower.getPose();

        // Calculate errors
        double xError = finalPose.getX() - startPose.getX();
        double yError = finalPose.getY() - startPose.getY();
        double positionError = Math.hypot(xError, yError);
        double headingError = Math.toDegrees(normalizeAngle(finalPose.getHeading() - startPose.getHeading()));

        // Calculate accuracy percentage (100% = perfect, 0% = very bad)
        double positionAccuracy = Math.max(0, 100 - (positionError / SQUARE_SIZE * 100));

        // Display results
        multiTelemetry.clear();
        multiTelemetry.addLine("╔════════════════════════════════════════╗");
        multiTelemetry.addLine("║     SQUARE TEST COMPLETE ✓             ║");
        multiTelemetry.addLine("╚════════════════════════════════════════╝");
        multiTelemetry.addLine();

        multiTelemetry.addLine("📊 ACCURACY RESULTS:");
        multiTelemetry.addLine("─────────────────────────────────────────");
        multiTelemetry.addData("Position Error", "%.2f inches", positionError);
        multiTelemetry.addData("  • X Error", "%.2f inches", xError);
        multiTelemetry.addData("  • Y Error", "%.2f inches", yError);
        multiTelemetry.addData("Heading Error", "%.2f°", headingError);
        multiTelemetry.addLine();

        multiTelemetry.addData("Position Accuracy", "%.1f%%", positionAccuracy);
        multiTelemetry.addLine();

        // Rating system
        String rating;
        String emoji;
        if (positionError < 1.0) {
            rating = "EXCELLENT";
            emoji = "🌟";
        } else if (positionError < 2.0) {
            rating = "GOOD";
            emoji = "✅";
        } else if (positionError < 4.0) {
            rating = "FAIR";
            emoji = "⚠️";
        } else {
            rating = "NEEDS TUNING";
            emoji = "❌";
        }

        multiTelemetry.addLine("─────────────────────────────────────────");
        multiTelemetry.addData("Rating", emoji + " " + rating);
        multiTelemetry.addLine("─────────────────────────────────────────");
        multiTelemetry.addLine();

        multiTelemetry.addLine("📍 POSITION DATA:");
        multiTelemetry.addData("Start Position", "X=%.2f, Y=%.2f, H=%.1f°",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        multiTelemetry.addData("Final Position", "X=%.2f, Y=%.2f, H=%.1f°",
                finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading()));
        multiTelemetry.addLine();

        // Recommendations
        multiTelemetry.addLine("💡 RECOMMENDATIONS:");
        if (positionError > 2.0) {
            multiTelemetry.addLine("  • Check Pinpoint odometry calibration");
            multiTelemetry.addLine("  • Verify ticks-to-inches multipliers");
            multiTelemetry.addLine("  • Run ForwardTuner and LateralTuner");
        }
        if (Math.abs(headingError) > 5.0) {
            multiTelemetry.addLine("  • Check yaw scalar in LConstants.java");
            multiTelemetry.addLine("  • Run TurnTuner for heading calibration");
        }
        if (positionError < 1.0 && Math.abs(headingError) < 2.0) {
            multiTelemetry.addLine("  • Localization is well-tuned! ✓");
            multiTelemetry.addLine("  • Ready for autonomous competition");
        }

        multiTelemetry.update();

        // Hold results for 10 seconds
        sleep(10000);
    }

    /**
     * Drive to a target pose and display progress
     */
    private void driveTo(Pose target, String description, Pose from) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(from),
                        new Point(target)))
                .setLinearHeadingInterpolation(from.getHeading(), target.getHeading())
                .build();

        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            Pose currentPose = follower.getPose();
            double distanceToTarget = Math.hypot(
                    currentPose.getX() - target.getX(),
                    currentPose.getY() - target.getY()
            );

            multiTelemetry.addLine("=== " + description + " ===");
            multiTelemetry.addData("Status", "🚗 Driving");
            multiTelemetry.addLine();
            multiTelemetry.addData("Current X", "%.2f inches", currentPose.getX());
            multiTelemetry.addData("Current Y", "%.2f inches", currentPose.getY());
            multiTelemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
            multiTelemetry.addLine();
            multiTelemetry.addData("Target X", "%.2f inches", target.getX());
            multiTelemetry.addData("Target Y", "%.2f inches", target.getY());
            multiTelemetry.addLine();
            multiTelemetry.addData("Distance Remaining", "%.2f inches", distanceToTarget);

            // Progress bar
            double totalDistance = Math.hypot(target.getX() - from.getX(), target.getY() - from.getY());
            double progress = Math.max(0, Math.min(100, ((totalDistance - distanceToTarget) / totalDistance) * 100));
            String progressBar = createProgressBar((int)progress);
            multiTelemetry.addData("Progress", progressBar + " " + (int)progress + "%");

            multiTelemetry.update();
        }
    }

    /**
     * Wait at corner with countdown
     */
    private void waitAtCorner(int cornerNumber) throws InterruptedException {
        if (CORNER_WAIT_TIME <= 0) return;

        long waitTimeMs = (long)(CORNER_WAIT_TIME * 1000);
        long waitStartTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - waitStartTime) < waitTimeMs) {
            follower.update();

            long elapsed = System.currentTimeMillis() - waitStartTime;
            double remaining = (waitTimeMs - elapsed) / 1000.0;

            Pose currentPose = follower.getPose();

            multiTelemetry.addLine("=== Corner " + cornerNumber + " Reached ===");
            multiTelemetry.addData("Status", "⏸️ Pausing");
            multiTelemetry.addLine();
            multiTelemetry.addData("Current X", "%.2f inches", currentPose.getX());
            multiTelemetry.addData("Current Y", "%.2f inches", currentPose.getY());
            multiTelemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
            multiTelemetry.addLine();
            multiTelemetry.addData("Wait Time Remaining", "%.1f seconds", remaining);
            multiTelemetry.update();
        }
    }

    /**
     * Create a visual progress bar
     */
    private String createProgressBar(int percent) {
        int filled = percent / 5; // 20 characters total
        int empty = 20 - filled;

        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < filled; i++) {
            bar.append("█");
        }
        for (int i = 0; i < empty; i++) {
            bar.append("░");
        }
        bar.append("]");

        return bar.toString();
    }

    /**
     * Normalize angle to [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}