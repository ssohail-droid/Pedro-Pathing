//package pedroPathing.SubSystem.Opmodes;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.ShooterSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//
//@TeleOp(name = "Blue TeleOp (Right POV)", group = "Subsystems")
//public class RedTeleOp extends OpMode {
//
//    private Follower follower;
//    private final Pose startPose = new Pose(134.3, 110.3, 0);
//    private final Pose redTarget = new Pose(108, 108, Math.toRadians(225)); // RED auto point
//
//    private boolean navigating = false;
//    private boolean waitingForContinue = false;
//    private Pose currentTarget = null;
//
//    private IntakeSubsystem intake;
//    private ShooterSubsystem shooter;
//    private TransferSubsystem transfer;
//    private ServoSubsystem servos;
//
//    private boolean aPressed = false, bPressed = false;
//    private boolean intakeFeedToggle = false, intakeFeedPressed = false;
//    private boolean shooterToggle = false, shooterPressed = false;
//    private static final double SHOOTER_REVERSE_POWER = 0;
//
//    private static final double POSITION_TOLERANCE = 0.1;
//    private static final double HEADING_TOLERANCE = 0.1;
//
//    @Override
//    public void init() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
//        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
//        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");
//        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
//        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");
//
//        intake = new IntakeSubsystem(intakeMotor);
//        shooter = new ShooterSubsystem(shooter1, shooter2);
//        transfer = new TransferSubsystem(feedMotor);
//        servos = new ServoSubsystem(pushServo);
//
//        telemetry.addLine("Red Alliance TeleOp (Right POV)");
//        telemetry.addLine("A: Go to Red Auto Point");
//        telemetry.addLine("B: Cancel or Continue");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//    }
//
//    @Override
//    public void loop() {
//        // ===== WAIT MODE =====
//        if (waitingForContinue) {
//            telemetry.addLine("Arrived at RED Target — Press B to continue");
//            follower.setTeleOpMovementVectors(0, 0, 0, false);
//            follower.update();
//
//            if (gamepad1.b && !bPressed) {
//                waitingForContinue = false;
//                currentTarget = null;
//                follower.startTeleopDrive();
//                bPressed = true;
//            } else if (!gamepad1.b) bPressed = false;
//            telemetry.update();
//            return;
//        }
//
//        // ===== AUTONAV =====
//        if (gamepad1.a && !aPressed && !navigating) {
//            startNavigation(redTarget);
//            aPressed = true;
//        } else if (!gamepad1.a) aPressed = false;
//
//        if (gamepad1.b && !bPressed && navigating) {
//            cancelNavigation();
//            bPressed = true;
//        } else if (!gamepad1.b) bPressed = false;
//
//        Pose pose = follower.getPose();
//        if (pose == null) pose = startPose;
//
//        if (navigating) {
//            follower.update();
//            if (arrivedAtTarget()) {
//                navigating = false;
//                waitingForContinue = true;
//                follower.breakFollowing();
//            }
//        } else {
//            // ===== MANUAL DRIVE (Right POV) =====
//            double rotatedX = gamepad1.left_stick_x;
//            double rotatedY = -gamepad1.left_stick_y;
//            follower.setTeleOpMovementVectors(rotatedX, rotatedY, -gamepad1.right_stick_x, false);
//            follower.update();
//        }
//
//        // ===== GAMEPAD 2 CONTROLS =====
//        if (gamepad2.a && !intakeFeedPressed) {
//            intakeFeedToggle = !intakeFeedToggle;
//            intakeFeedPressed = true;
//            if (intakeFeedToggle) { intake.start(); transfer.start(); }
//            else { intake.stop(); transfer.stop(); }
//        } else if (!gamepad2.a) intakeFeedPressed = false;
//
//        if (gamepad2.right_trigger > 0.5 && !shooterPressed) {
//            shooterToggle = !shooterToggle;
//            shooterPressed = true;
//        } else if (gamepad2.right_trigger <= 0.5) shooterPressed = false;
//
//        if (shooterToggle) shooter.spinUp();
//        else { shooter.stop(); shooter.setIdlePower(SHOOTER_REVERSE_POWER); }
//
//        if (gamepad2.share) { intake.reverse(); transfer.reverse(); }
//
//        if (gamepad2.y) servos.engagePush();
//        else servos.retractPush();
//
//        // ===== TELEMETRY =====
//        telemetry.addLine(navigating ? "AutoNav: RED Target" : "Manual Drive (Right POV)");
//        telemetry.addData("X", "%.2f", pose.getX());
//        telemetry.addData("Y", "%.2f", pose.getY());
//        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
//        telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
//        telemetry.addData("Shooter", shooter.isSpinning() ? "Spinning" : "Idle");
//        telemetry.update();
//    }
//
//    private void startNavigation(Pose target) {
//        currentTarget = target;
//        navigating = true;
//        follower.followPath(follower.pathBuilder()
//                .addPath(new com.pedropathing.pathgen.BezierLine(
//                        new com.pedropathing.pathgen.Point(follower.getPose()),
//                        new com.pedropathing.pathgen.Point(target)))
//                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
//                .build());
//    }
//
//    private void cancelNavigation() {
//        navigating = false;
//        currentTarget = null;
//        waitingForContinue = false;
//        follower.breakFollowing();
//        follower.startTeleopDrive();
//    }
//
//    private boolean arrivedAtTarget() {
//        if (currentTarget == null) return false;
//        Pose pose = follower.getPose();
//        if (pose == null) pose = startPose;
//        double dx = currentTarget.getX() - pose.getX();
//        double dy = currentTarget.getY() - pose.getY();
//        double dist = Math.hypot(dx, dy);
//        double headingError = Math.abs(currentTarget.getHeading() - pose.getHeading());
//        while (headingError > Math.PI) headingError -= 2 * Math.PI;
//        while (headingError < -Math.PI) headingError += 2 * Math.PI;
//        return dist < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE;
//    }
//}
