//package pedroPathing.SubSystem.Opmodes;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.ShooterSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//import pedroPathing.SubSystem.VisionSubsystem;
//
///**
// * Field-centric teleop with all subsystems integrated + autonomous point navigation.
// *
// * Controls:
// * GAMEPAD 1 (Driver):
// * - Left Stick Y: Forward/Backward
// * - Left Stick X: Strafe Left/Right
// * - Right Stick X: Rotate
// * - A: Go to Target Point 1 (autonomous)
// * - B: Cancel autonomous navigation / return to manual
// * - X: Go to Target Point 2
// * - Y: Go to Target Point 3
// * - Left Trigger: Start Intake
// * - Left Bumper: Stop Intake
// * - Right Trigger: Start Transfer
// * - Right Bumper: Stop Transfer
// * - Dpad Up: Get Vision Alignment Data
// *
// * GAMEPAD 2 (Operator):
// * - A: Spin Up Shooter
// * - B: Set Shooter to Idle
// * - X: Stop Shooter
// * - Y: Engage Hold Servo
// * - Dpad Down: Release Hold Servo
// * - Dpad Left: Engage Push Servo
// * - Dpad Right: Retract Push Servo
// */
//@TeleOp(name = "Field-Centric with Auto Navigation", group = "Subsystems")
//public class FieldCentricDriveWithAutoMoveOpMode extends OpMode {
//
//    // PedroPathing follower for field-centric drive
//    private Follower follower;
//    private final Pose startPose = new Pose(0, 0, 0);
//
//    // Target poses for autonomous navigation (adjust coordinates to your field)
//    private final Pose targetPoint1 = new Pose(60, 36, 0);      // Example: scoring position
//    private final Pose targetPoint2 = new Pose(24, 24, Math.toRadians(90));  // Example: intake station
//    private final Pose targetPoint3 = new Pose(48, 12, Math.toRadians(-45)); // Example: substation
//
//    // Navigation state
//    private boolean navigatingToPoint = false;
//    private Pose currentTarget = null;
//
//    // Tolerances for "arrived" detection
//    private static final double POSITION_TOLERANCE = 2.0;  // inches
//    private static final double HEADING_TOLERANCE = Math.toRadians(5); // 5 degrees
//
//    // Subsystems
//    private IntakeSubsystem intake;
//    private ShooterSubsystem shooter;
//    private TransferSubsystem transfer;
//    private ServoSubsystem servos;
//    private VisionSubsystem vision;
//
//    private static final double TARGET_DISTANCE = 0.25;
//
//    // Vision components
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//
//    // Button debounce
//    private boolean aPressed = false;
//    private boolean bPressed = false;
//    private boolean xPressed = false;
//    private boolean yPressed = false;
//
//    @Override
//    public void init() {
//        // Initialize PedroPathing constants and follower
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        // Initialize mechanism hardware
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
//        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
//        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");
//        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
//        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
//        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");
//
//        // Initialize vision
//        aprilTag = new AprilTagProcessor.Builder()
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .build();
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .build();
//
//        // Initialize subsystems
//        intake = new IntakeSubsystem(intakeMotor);
//        shooter = new ShooterSubsystem(shooter1, shooter2);
//        transfer = new TransferSubsystem(feedMotor);
//        servos = new ServoSubsystem(holdServo, pushServo);
//        vision = new VisionSubsystem(visionPortal, aprilTag);
//
//        telemetry.addLine("Field-Centric Drive with Auto Navigation");
//        telemetry.addLine("GP1 A/X/Y: Navigate to points");
//        telemetry.addLine("GP1 B: Cancel navigation");
//        telemetry.update();
//    }
//
//    @Override
//    public void init_loop() {
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//    }
//
//    @Override
//    public void loop() {
//
//        // ===== AUTONOMOUS NAVIGATION CONTROL =====
//        // A: Navigate to Point 1
//        if (gamepad1.a && !aPressed && !navigatingToPoint) {
//            startNavigationToPoint(targetPoint1);
//            aPressed = true;
//        } else if (!gamepad1.a) {
//            aPressed = false;
//        }
//
//        // X: Navigate to Point 2
//        if (gamepad1.x && !xPressed && !navigatingToPoint) {
//            startNavigationToPoint(targetPoint2);
//            xPressed = true;
//        } else if (!gamepad1.x) {
//            xPressed = false;
//        }
//
//        // Y: Navigate to Point 3
//        if (gamepad1.y && !yPressed && !navigatingToPoint) {
//            startNavigationToPoint(targetPoint3);
//            yPressed = true;
//        } else if (!gamepad1.y) {
//            yPressed = false;
//        }
//
//        // B: Cancel navigation and return to manual control
//        if (gamepad1.b && !bPressed && navigatingToPoint) {
//            cancelNavigation();
//            bPressed = true;
//        } else if (!gamepad1.b) {
//            bPressed = false;
//        }
//
//        // ===== DRIVE CONTROL =====
//        if (navigatingToPoint) {
//            // Autonomous navigation mode
//            follower.update();
//
//            // Check if we've arrived at the target
//            if (hasArrivedAtTarget()) {
//                navigatingToPoint = false;
//                currentTarget = null;
//                follower.startTeleopDrive();
//                telemetry.addLine(">>> ARRIVED AT TARGET <<<");
//            }
//
//            // Display navigation telemetry
//            double dx = currentTarget.getX() - follower.getPose().getX();
//            double dy = currentTarget.getY() - follower.getPose().getY();
//            double dist = Math.hypot(dx, dy);
//            double headingError = Math.toDegrees(
//                    Math.abs(currentTarget.getHeading() - follower.getPose().getHeading())
//            );
//
//            telemetry.addLine("=== NAVIGATING TO POINT ===");
//            telemetry.addData("Target X", "%.1f", currentTarget.getX());
//            telemetry.addData("Target Y", "%.1f", currentTarget.getY());
//            telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(currentTarget.getHeading()));
//            telemetry.addData("Distance", "%.2f in", dist);
//            telemetry.addData("Heading Error", "%.1f°", headingError);
//            telemetry.addLine("Press B to cancel");
//
//        } else {
//            // Manual field-centric drive mode
//            follower.setTeleOpMovementVectors(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    false
//            );
//            follower.update();
//        }
//
//        // ===== GAMEPAD 1: INTAKE & TRANSFER =====
//        // Left trigger: Start intake
//        if (gamepad1.left_trigger > 0.5) {
//            intake.start();
//        }
//        // Left bumper: Stop intake
//        if (gamepad1.left_bumper) {
//            intake.stop();
//        }
//
//        // Right trigger: Start transfer
//        if (gamepad1.right_trigger > 0.5) {
//            transfer.start();
//        }
//        // Right bumper: Stop transfer
//        if (gamepad1.right_bumper) {
//            transfer.stop();
//        }
//
//        // ===== GAMEPAD 1: VISION =====
//        // Dpad Up: Get vision alignment data
//        if (gamepad1.dpad_up) {
//            VisionSubsystem.AlignmentData data = vision.getAlignmentData(TARGET_DISTANCE);
//            if (data != null) {
//                telemetry.addLine("=== VISION ALIGNMENT ===");
//                telemetry.addData("Tag ID", data.tagID);
//                telemetry.addData("Error X", "%.2f", data.errorX);
//                telemetry.addData("Turn Correction", "%.3f", data.turnCorrection);
//            } else {
//                telemetry.addLine("No AprilTag detected");
//            }
//        }
//
//        // ===== GAMEPAD 2: SHOOTER =====
//        // A: Spin up shooter
//        if (gamepad2.a) {
//            shooter.spinUp();
//        }
//        // B: Set shooter to idle
//        if (gamepad2.b) {
//            shooter.setIdle();
//        }
//        // X: Stop shooter
//        if (gamepad2.x) {
//            shooter.stop();
//        }
//
//        // ===== GAMEPAD 2: SERVOS =====
//        // Y: Engage hold servo
//        if (gamepad2.y) {
//            servos.engageHold();
//        }
//        // Dpad Down: Release hold servo
//        if (gamepad2.dpad_down) {
//            servos.releaseHold();
//        }
//        // Dpad Left: Engage push servo
//        if (gamepad2.dpad_left) {
//            servos.engagePush();
//        }
//        // Dpad Right: Retract push servo
//        if (gamepad2.dpad_right) {
//            servos.retractPush();
//        }
//
//        // ===== TELEMETRY =====
//        if (!navigatingToPoint) {
//            telemetry.addLine("=== MANUAL DRIVE MODE ===");
//        }
//        telemetry.addLine("=== ROBOT POSE ===");
//        telemetry.addData("X", "%.2f", follower.getPose().getX());
//        telemetry.addData("Y", "%.2f", follower.getPose().getY());
//        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
//
//        telemetry.addLine("=== MECHANISMS ===");
//        telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
//        telemetry.addData("Transfer", transfer.isRunning() ? "Running" : "Stopped");
//        telemetry.addData("Shooter", shooter.isSpinning() ? "Spinning" : "Idle/Stopped");
//
//        telemetry.update();
//    }
//
//    /**
//     * Start autonomous navigation to a target pose
//     */
//    private void startNavigationToPoint(Pose target) {
//        currentTarget = target;
//        navigatingToPoint = true;
//        follower.followPath(follower.pathBuilder()
//                .addPath(new com.pedropathing.pathgen.BezierLine(
//                        new com.pedropathing.pathgen.Point(follower.getPose()),
//                        new com.pedropathing.pathgen.Point(target)
//                ))
//                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
//                .build());
//    }
//
//    /**
//     * Cancel autonomous navigation and return to manual control
//     */
//    private void cancelNavigation() {
//        navigatingToPoint = false;
//        currentTarget = null;
//        follower.breakFollowing();
//        follower.startTeleopDrive();
//    }
//
//    /**
//     * Check if robot has arrived at the current target
//     */
//    private boolean hasArrivedAtTarget() {
//        if (currentTarget == null) return false;
//
//        double dx = currentTarget.getX() - follower.getPose().getX();
//        double dy = currentTarget.getY() - follower.getPose().getY();
//        double distanceError = Math.hypot(dx, dy);
//
//        double headingError = Math.abs(currentTarget.getHeading() - follower.getPose().getHeading());
//        // Normalize heading error to [-π, π]
//        while (headingError > Math.PI) headingError -= 2 * Math.PI;
//        while (headingError < -Math.PI) headingError += 2 * Math.PI;
//        headingError = Math.abs(headingError);
//
//        return distanceError < POSITION_TOLERANCE && headingError < HEADING_TOLERANCE;
//    }
//
//    @Override
//    public void stop() {
//        // Clean up vision resources
//        if (vision != null) {
//            vision.close();
//        }
//    }
//}
