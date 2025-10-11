package pedroPathing.SubSystem.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;

/**
 * Field-centric teleop with all subsystems integrated + autonomous point navigation.
 *
 * Controls:
 * GAMEPAD 1 (Driver):
 * - Left Stick Y: Forward/Backward
 * - Left Stick X: Strafe Left/Right
 * - Right Stick X: Rotate
 * - A: Go to halfway bottom-left corner
 * - X: Go to halfway bottom-right corner
 * - Y: Go to top middle (example)
 * - B: Cancel autonomous navigation or continue after arrival (ADDED)
 * - Left Trigger: Start Intake
 * - Left Bumper: Stop Intake
 * - Right Trigger: Start Transfer
 * - Right Bumper: Stop Transfer
 *
 * GAMEPAD 2 (Operator):
 * - A: Spin Up Shooter
 * - B: Set Shooter to Idle
 * - X: Stop Shooter
 * - Y: Toggle Push Servo (ON/OFF)
 */
@TeleOp(name = "Field-Centric with Auto Navigation Pause Until B", group = "Subsystems")
public class FieldCentricDriveAutoNavNoWebcam extends OpMode {

    // PedroPathing follower for field-centric drive
    private Follower follower;

    // === FIELD COORDINATES ===
    // 144x144 inches total, 72x72 center
    // (0,0) bottom-left, (144,144) top-right

    // Start pose: center, facing south (toward bottom wall)
    private final Pose startPose = new Pose(72, 72, 0);

    // Target poses for autonomous navigation
    private final Pose targetPoint1 = new Pose(90, 90, Math.toRadians(225));  // Halfway to bottom-left, +45° heading
    private final Pose targetPoint2 = new Pose(90, 54, Math.toRadians(135)); // Halfway to bottom-right, -45° heading
    private final Pose targetPoint3 = new Pose(72, 72, 0);   // Back to center

    // Navigation state
    private boolean navigatingToPoint = false;
    private boolean waitingForContinue = false; // ADDED
    private Pose currentTarget = null;

    // Tolerances for "arrived" detection
    private static final double POSITION_TOLERANCE = 0.2;  // inches
    private static final double HEADING_TOLERANCE = 0.1;

    // Subsystems
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;

    // Button debounce
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;

    @Override
    public void init() {
        // Initialize PedroPathing constants and follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize mechanism hardware
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        // Initialize subsystems
        intake = new IntakeSubsystem(intakeMotor);
        shooter = new ShooterSubsystem(shooter1, shooter2);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(pushServo);

        telemetry.addLine("Field-Centric Drive with Auto Navigation");
        telemetry.addLine("A/X/Y: Navigate to set points");
        telemetry.addLine("B: Cancel navigation or Continue after arrival (ADDED)");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // ===== WAITING MODE (ADDED) =====
        if (waitingForContinue) {
            telemetry.addLine("=== ARRIVED AT TARGET ===");
            telemetry.addLine("Robot stopped. Press B to continue...");
            follower.setTeleOpMovementVectors(0, 0, 0, false);
            follower.update();

            // Wait until B is pressed to continue manual control
            if (gamepad1.b && !bPressed) {
                waitingForContinue = false;
                currentTarget = null; // MOVED HERE ✅
                follower.startTeleopDrive();
                telemetry.addLine(">>> CONTINUED <<<");
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }

            telemetry.update();
            return; // Stop rest of loop while waiting
        }

        // ===== AUTONOMOUS NAVIGATION CONTROL =====
        if (gamepad1.a && !aPressed && !navigatingToPoint) {
            startNavigationToPoint(targetPoint1);
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        if (gamepad1.x && !xPressed && !navigatingToPoint) {
            startNavigationToPoint(targetPoint2);
            xPressed = true;
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        if (gamepad1.y && !yPressed && !navigatingToPoint) {
            startNavigationToPoint(targetPoint3);
            yPressed = true;
        } else if (!gamepad1.y) {
            yPressed = false;
        }

        if (gamepad1.b && !bPressed && navigatingToPoint) {
            cancelNavigation();
            bPressed = true;
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        // ===== DRIVE CONTROL =====
        Pose robotPose = follower.getPose();
        if (robotPose == null) robotPose = startPose;

        if (navigatingToPoint) {
            // Autonomous navigation mode
            follower.update();

            if (hasArrivedAtTarget()) {
                navigatingToPoint = false;
                waitingForContinue = true; // ADDED
                follower.breakFollowing(); // ADDED - stop movement
                telemetry.addLine(">>> ARRIVED AT TARGET <<<");
                telemetry.addLine("Robot stopped. Press B to continue...");
            }

            // Display navigation telemetry only if target is still set
            if (currentTarget != null) {
                double dx = currentTarget.getX() - robotPose.getX();
                double dy = currentTarget.getY() - robotPose.getY();
                double dist = Math.hypot(dx, dy);
                double headingError = Math.toDegrees(
                        Math.abs(currentTarget.getHeading() - robotPose.getHeading())
                );

                telemetry.addLine("=== NAVIGATING TO POINT ===");
                telemetry.addData("Target X", "%.1f", currentTarget.getX());
                telemetry.addData("Target Y", "%.1f", currentTarget.getY());
                telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(currentTarget.getHeading()));
                telemetry.addData("Distance", "%.2f in", dist);
                telemetry.addData("Heading Error", "%.1f°", headingError);
                telemetry.addLine("Press B to cancel");
            }

        } else {
            // Manual field-centric drive mode
            follower.setTeleOpMovementVectors(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // ===== GAMEPAD 2: INTAKE & TRANSFER =====
        if (gamepad2.left_trigger > 0.5) {
            intake.start();
        }
        if (gamepad2.left_bumper) {
            intake.stop();
        }

        if (gamepad2.right_trigger > 0.5) {
            transfer.start();
        }
        if (gamepad2.right_bumper) {
            transfer.stop();
        }

        // ===== GAMEPAD 2: SHOOTER =====
        if (gamepad2.a) {
            shooter.spinUp();
        }
        if (gamepad2.b) {
            shooter.setIdle();
        }
        if (gamepad2.x) {
            shooter.stop();
        }

        // ===== GAMEPAD 2: PUSH SERVO TOGGLE =====
        if (gamepad2.y && !yPressed) {
            servos.togglePush();
            yPressed = true;
        } else if (!gamepad2.y) {
            yPressed = false;
        }

        // ===== TELEMETRY =====
        if (!navigatingToPoint && !waitingForContinue) {
            telemetry.addLine("=== MANUAL DRIVE MODE ===");
        }
        telemetry.addLine("=== ROBOT POSE ===");
        telemetry.addData("X", "%.2f", robotPose.getX());
        telemetry.addData("Y", "%.2f", robotPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));

        telemetry.addLine("=== MECHANISMS ===");
        telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Transfer", transfer.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Shooter", shooter.isSpinning() ? "Spinning" : "Idle/Stopped");
        telemetry.addData("Push Servo", servos.isPushActive() ? "ENGAGED" : "RETRACTED");

        telemetry.update();
    }

    // ===== NAVIGATION HELPERS =====
    private void startNavigationToPoint(Pose target) {
        currentTarget = target;
        navigatingToPoint = true;
        follower.followPath(follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(follower.getPose()),
                        new com.pedropathing.pathgen.Point(target)
                ))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build());
    }

    private void cancelNavigation() {
        navigatingToPoint = false;
        currentTarget = null;
        waitingForContinue = false; // ADDED
        follower.breakFollowing();
        follower.startTeleopDrive();
    }

    private boolean hasArrivedAtTarget() {
        if (currentTarget == null) return false;

        Pose robotPose = follower.getPose();
        if (robotPose == null) robotPose = startPose;

        double dx = currentTarget.getX() - robotPose.getX();
        double dy = currentTarget.getY() - robotPose.getY();
        double distanceError = Math.hypot(dx, dy);

        double headingError = Math.abs(currentTarget.getHeading() - robotPose.getHeading());
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        headingError = Math.abs(headingError);

        return distanceError < POSITION_TOLERANCE && headingError < HEADING_TOLERANCE;
    }

    @Override
    public void stop() {
        // No webcam cleanup required
    }
}
