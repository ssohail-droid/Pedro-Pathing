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
 * Adjusted for side POV (right side of the field).
 *
 * Controls:
 * GAMEPAD 1 (Driver):
 * - Left Stick Y: Forward/Backward (rotated for right-side POV)
 * - Left Stick X: Strafe Left/Right
 * - Right Stick X: Rotate
 * - A: Go to halfway bottom-left corner
 * - X: Go to halfway bottom-right corner
 * - B: Cancel autonomous navigation or continue after arrival
 *
 * GAMEPAD 2 (Operator):
 * - A: Toggle Intake and Feed
 * - Right Trigger: Toggle Shooter
 * - Y: Hold for Push Servo
 */
@TeleOp(name = "use this (Right-Side POV)", group = "Subsystems")
public class FieldCentricDriveAutoNavRotated extends OpMode {

    // PedroPathing follower for field-centric drive
    private Follower follower;

    // Field setup
    private final Pose startPose = new Pose(72, 72, 0);
    private final Pose targetPoint1 = new Pose(108, 104, Math.toRadians(225));
    private final Pose targetPoint2 = new Pose(108, 40, Math.toRadians(135));

    private boolean navigatingToPoint = false;
    private boolean waitingForContinue = false;
    private Pose currentTarget = null;

    private static final double POSITION_TOLERANCE = 0.1;
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

    // Toggles
    private boolean intakeFeedToggle = false;
    private boolean intakeFeedTogglePressed = false;
    private boolean shooterToggle = false;
    private boolean shooterTogglePressed = false;

    private static final double SHOOTER_REVERSE_POWER = -0.1;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        intake = new IntakeSubsystem(intakeMotor);
        shooter = new ShooterSubsystem(shooter1, shooter2);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(pushServo);

        telemetry.addLine("Field-Centric Drive (Right-Side POV)");
        telemetry.addLine("A/X: Navigate to set points");
        telemetry.addLine("B: Cancel or Continue after arrival");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // ===== WAITING MODE =====
        if (waitingForContinue) {
            telemetry.addLine("=== ARRIVED AT TARGET ===");
            telemetry.addLine("Press B to continue...");
            follower.setTeleOpMovementVectors(0, 0, 0, false);
            follower.update();

            if (gamepad1.b && !bPressed) {
                waitingForContinue = false;
                currentTarget = null;
                follower.startTeleopDrive();
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }

            telemetry.update();
            return;
        }

        // ===== AUTONOMOUS NAVIGATION =====
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

        if (gamepad1.b && !bPressed && navigatingToPoint) {
            cancelNavigation();
            bPressed = true;
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        Pose robotPose = follower.getPose();
        if (robotPose == null) robotPose = startPose;

        if (navigatingToPoint) {
            follower.update();

            if (hasArrivedAtTarget()) {
                navigatingToPoint = false;
                waitingForContinue = true;
                follower.breakFollowing();
            }

        } else {
            // ===== MANUAL DRIVE CONTROL (rotated for right-side POV) =====
            // Rotate joystick inputs by -90° so forward on the stick moves away from driver station (right-side perspective)
            double rotatedX = -gamepad1.left_stick_y; // Forward/backward from POV
            double rotatedY = gamepad1.left_stick_x;  // Strafe left/right from POV

            follower.setTeleOpMovementVectors(
                    rotatedX,
                    rotatedY,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // ===== GAMEPAD 2 CONTROLS =====
        if (gamepad2.a && !intakeFeedTogglePressed) {
            intakeFeedToggle = !intakeFeedToggle;
            intakeFeedTogglePressed = true;

            if (intakeFeedToggle) {
                intake.start();
                transfer.start();
            } else {
                intake.stop();
                transfer.stop();
            }
        } else if (!gamepad2.a) {
            intakeFeedTogglePressed = false;
        }

        if (gamepad2.right_trigger > 0.5 && !shooterTogglePressed) {
            shooterToggle = !shooterToggle;
            shooterTogglePressed = true;
        } else if (gamepad2.right_trigger <= 0.5) {
            shooterTogglePressed = false;
        }

        if (shooterToggle) {
            shooter.spinUp();
        } else {
            shooter.stop();
            shooter.setIdlePower(SHOOTER_REVERSE_POWER);
        }

        if (gamepad2.share) {
            intake.reverse();
            transfer.reverse();
        }

        if (gamepad2.y) {
            servos.engagePush();
        } else {
            servos.retractPush();
        }

        // ===== TELEMETRY =====
        telemetry.addLine(navigatingToPoint ? "=== AUTO NAV ===" : "=== MANUAL DRIVE (Right POV) ===");
        telemetry.addData("X", "%.2f", robotPose.getX());
        telemetry.addData("Y", "%.2f", robotPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Shooter", shooter.isSpinning() ? "Spinning" : "Idle");
        telemetry.update();
    }

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
        waitingForContinue = false;
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
        // No webcam cleanup needed
    }
}
