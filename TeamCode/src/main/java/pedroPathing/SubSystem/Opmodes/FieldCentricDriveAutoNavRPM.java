//package pedroPathing.SubSystem.Opmodes;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.dashboard.config.Config;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//
//@Disabled
//@Config
//@TeleOp(name = "Field-Centric AutoNav + RPM Shooter Stable", group = "Subsystems")
//public class FieldCentricDriveAutoNavRPM extends OpMode {
//
//    private Follower follower;
//
//    private final Pose startPose = new Pose(72, 72, 0);
//    private final Pose targetPoint1 = new Pose(108, 108, Math.toRadians(225));
//    private final Pose targetPoint2 = new Pose(108, 36, Math.toRadians(135));
//
//    private boolean navigatingToPoint = false;
//    private boolean waitingForContinue = false;
//    private Pose currentTarget = null;
//
//    private static final double POSITION_TOLERANCE = 0.2;
//    private static final double HEADING_TOLERANCE = 0.1;
//
//    private IntakeSubsystem intake;
//    private TransferSubsystem transfer;
//    private ServoSubsystem servos;
//
//    private DcMotorEx shooter1, shooter2;
//
//    // Shooter PIDF tunable parameters
//    public static double kP = 0.0007;
//    public static double kI = 0.00001;
//    public static double kD = 0.00005;
//    public static double kF = 0.00015;
//
//    public static double SHOOTER_TARGET_RPM = 3000;
//    public static double SHOOTER_IDLE_RPM = 50;
//    public static double MAX_POWER = 1.0;
//    public static double SHOOTER_RPM_TOLERANCE = 50;
//
//    private double shooterIntegral = 0;
//    private double lastError = 0;
//    private final ElapsedTime pidTimer = new ElapsedTime();
//    private double lastPosition = 0;
//    private double lastTime = 0;
//    private boolean shooterSpinning = false;
//
//    // Velocity filter
//    private double filteredRPM = 0;
//    private double alpha = 0.2; // smoothing factor
//
//    private FtcDashboard dashboard;
//
//    private boolean aPressed = false, bPressed = false, xPressed = false;
//    private boolean intakeFeedToggle = false, intakeFeedPressed = false;
//    private boolean shooterToggle = false, shooterTogglePressed = false;
//
//    @Override
//    public void init() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
//        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter_motor_1");
//        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter_motor_2");
//        DcMotorEx feedMotor = hardwareMap.get(DcMotorEx.class, "feed_motor");
//        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");
//
//        intake = new IntakeSubsystem(intakeMotor);
//        transfer = new TransferSubsystem(feedMotor);
//        servos = new ServoSubsystem(pushServo);
//
//        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
//
//        dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//        lastPosition = shooter1.getCurrentPosition();
//        lastTime = pidTimer.seconds();
//
//        telemetry.addLine("Field-Centric Drive + RPM Shooter PID (Stable)");
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
//
//        // Waiting mode
//        if (waitingForContinue) {
//            telemetry.addLine("=== ARRIVED AT TARGET ===");
//            telemetry.addLine("Press B to continue...");
//            follower.setTeleOpMovementVectors(0, 0, 0, false);
//            follower.update();
//
//            if (gamepad1.b && !bPressed) {
//                waitingForContinue = false;
//                currentTarget = null;
//                follower.startTeleopDrive();
//                bPressed = true;
//            } else if (!gamepad1.b) bPressed = false;
//
//            telemetry.update();
//            return;
//        }
//
//        // AutoNav triggers
//        if (gamepad1.a && !aPressed && !navigatingToPoint) {
//            startNavigationToPoint(targetPoint1);
//            aPressed = true;
//        } else if (!gamepad1.a) aPressed = false;
//
//        if (gamepad1.x && !xPressed && !navigatingToPoint) {
//            startNavigationToPoint(targetPoint2);
//            xPressed = true;
//        } else if (!gamepad1.x) xPressed = false;
//
//        if (gamepad1.b && !bPressed && navigatingToPoint) {
//            cancelNavigation();
//            bPressed = true;
//        } else if (!gamepad1.b) bPressed = false;
//
//        Pose robotPose = follower.getPose();
//        if (robotPose == null) robotPose = startPose;
//
//        if (navigatingToPoint) {
//            follower.update();
//            if (hasArrivedAtTarget()) {
//                navigatingToPoint = false;
//                waitingForContinue = true;
//                follower.breakFollowing();
//            }
//        } else {
//            follower.setTeleOpMovementVectors(
//                    gamepad1.left_stick_y,
//                    gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    false
//            );
//            follower.update();
//        }
//
//        // Intake toggle
//        if (gamepad2.a && !intakeFeedPressed) {
//            intakeFeedToggle = !intakeFeedToggle;
//            intakeFeedPressed = true;
//        } else if (!gamepad2.a) intakeFeedPressed = false;
//
//        // Shooter toggle
//        if (gamepad2.right_trigger > 0.5 && !shooterTogglePressed) {
//            shooterToggle = !shooterToggle;
//            shooterTogglePressed = true;
//        } else if (gamepad2.right_trigger <= 0.5) shooterTogglePressed = false;
//
//        shooterSpinning = shooterToggle;
//
//        // Shooter PID update
//        double shooterPower = updateShooterPID();
//
//        // Run intake/transfer only when shooter at target RPM
//        if (intakeFeedToggle && isShooterAtTarget()) {
//            intake.start();
//            transfer.start();
//        } else {
//            intake.stop();
//            transfer.stop();
//        }
//
//        // Push servo
//        if (gamepad2.y) servos.engagePush();
//        else servos.retractPush();
//
//        // Telemetry
//        telemetry.addLine("=== SHOOTER PID ===");
//        telemetry.addData("Spinning", shooterSpinning);
//        telemetry.addData("Shooter Power", "%.3f", shooterPower);
//        telemetry.addData("Current RPM", "%.0f", getShooterRPM());
//        telemetry.addData("Target RPM", shooterSpinning ? SHOOTER_TARGET_RPM : SHOOTER_IDLE_RPM);
//        telemetry.addData("Error", "%.1f", (shooterSpinning ? SHOOTER_TARGET_RPM : SHOOTER_IDLE_RPM) - getShooterRPM());
//        telemetry.addData("At Target RPM", isShooterAtTarget());
//
//        telemetry.addLine("=== ROBOT POSE ===");
//        telemetry.addData("X", "%.2f", robotPose.getX());
//        telemetry.addData("Y", "%.2f", robotPose.getY());
//        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));
//        telemetry.update();
//    }
//
//    // ---------------- SHOOTER PID ----------------
//    private double updateShooterPID() {
//        double desiredRPM = shooterSpinning ? SHOOTER_TARGET_RPM : SHOOTER_IDLE_RPM;
//        double currentRPM = getShooterRPM();
//
//        double error = desiredRPM - currentRPM;
//        double currentTime = pidTimer.seconds();
//        double dt = currentTime - lastTime;
//        if (dt <= 0) dt = 0.02;
//
//        shooterIntegral += error * dt;
//        double derivative = (error - lastError) / dt;
//        lastError = error;
//        lastTime = currentTime;
//
//        double output = kP * error + kI * shooterIntegral + kD * derivative + kF * (desiredRPM / 6000.0);
//        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));
//
//        shooter1.setPower(output);
//        shooter2.setPower(output);
//
//        // Dashboard telemetry
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Shooter/TargetRPM", desiredRPM);
//        packet.put("Shooter/CurrentRPM", currentRPM);
//        packet.put("Shooter/Error", error);
//        packet.put("Shooter/Power", output);
//        dashboard.sendTelemetryPacket(packet);
//
//        return output;
//    }
//
//    private double getShooterRPM() {
//        double currentPos = shooter1.getCurrentPosition();
//        double currentTime = pidTimer.seconds();
//
//        double deltaPos = currentPos - lastPosition;
//        double deltaTime = currentTime - lastTime;
//        if (deltaTime <= 0) deltaTime = 0.02;
//
//        lastPosition = currentPos;
//        lastTime = currentTime;
//
//        double ticksPerRev = 28 * 20; // REV UltraPlanetary, 20:1 example
//        double revsPerSec = deltaPos / ticksPerRev / deltaTime;
//        double rpm = revsPerSec * 60;
//
//        // Low-pass filter
//        filteredRPM = alpha * rpm + (1 - alpha) * filteredRPM;
//        return filteredRPM;
//    }
//
//    private boolean isShooterAtTarget() {
//        double targetRPM = shooterSpinning ? SHOOTER_TARGET_RPM : SHOOTER_IDLE_RPM;
//        double currentRPM = getShooterRPM();
//        return Math.abs(targetRPM - currentRPM) <= SHOOTER_RPM_TOLERANCE;
//    }
//
//    // ---------------- AutoNav ----------------
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
//    private void cancelNavigation() {
//        navigatingToPoint = false;
//        currentTarget = null;
//        waitingForContinue = false;
//        follower.breakFollowing();
//        follower.startTeleopDrive();
//    }
//
//    private boolean hasArrivedAtTarget() {
//        if (currentTarget == null) return false;
//        Pose robotPose = follower.getPose();
//        if (robotPose == null) robotPose = startPose;
//
//        double dx = currentTarget.getX() - robotPose.getX();
//        double dy = currentTarget.getY() - robotPose.getY();
//        double distError = Math.hypot(dx, dy);
//
//        double headingError = Math.abs(currentTarget.getHeading() - robotPose.getHeading());
//        while (headingError > Math.PI) headingError -= 2 * Math.PI;
//        while (headingError < -Math.PI) headingError += 2 * Math.PI;
//
//        return distError < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE;
//    }
//
//    @Override
//    public void stop() {
//        shooter1.setPower(0);
//        shooter2.setPower(0);
//        intake.stop();
//        transfer.stop();
//    }
//}
