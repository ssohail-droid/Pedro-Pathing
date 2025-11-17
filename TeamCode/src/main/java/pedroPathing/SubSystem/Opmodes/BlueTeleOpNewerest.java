package pedroPathing.SubSystem.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.List;

@Config
@TeleOp(name = "BLUE TeleOp (Limelight + ML)", group = "Competition")
public class BlueTeleOpNewerest extends OpMode {

    public static boolean ENABLE_LIMELIGHT = true;
    public static boolean ENABLE_DISTANCE_SHOOTING = true;

    public static double LIMELIGHT_HEIGHT_INCHES = 13.0;
    public static double LIMELIGHT_ANGLE_DEGREES = 75.0;
    public static double LIMELIGHT_HORIZONTAL_OFFSET_INCHES = 3.0;
    public static double LIMELIGHT_FORWARD_OFFSET_INCHES = 6.5;

    public static double CLASSIFIER_HEIGHT_INCHES = 48.0;
    public static double DEPOT_HEIGHT_INCHES = 12.0;

    public static double RPM_PER_INCH = 5.0;
    public static double BASE_SHOOTING_DISTANCE = 24.0;
    public static double MIN_SHOOTING_RPM = 2400;
    public static double MAX_SHOOTING_RPM = 2400;

    public static double AUTO_AIM_ROTATION_GAIN = 0.03;
    public static double AUTO_AIM_STRAFE_GAIN = 0.02;
    public static double AUTO_AIM_TOLERANCE_DEGREES = 2.0;
    public static double AUTO_AIM_MAX_ROTATION = 0.5;
    public static double AUTO_AIM_MAX_STRAFE = 0.3;

    public static int GREEN_ARTIFACT_PIPELINE = 1;
    public static int PURPLE_ARTIFACT_PIPELINE = 2;
    public static double ARTIFACT_MIN_AREA = 0.5;

    public static boolean ENABLE_ADAPTIVE_RPM = true;
    public static double RPM_VOLTAGE_COEFFICIENT = 15.0;
    public static double BASE_RPM = 2400;

    public static boolean ENABLE_BATTERY_COMPENSATION = true;
    public static double MIN_VOLTAGE_THRESHOLD = 11.5;

    private final Pose startPose = new Pose(30, 124,Math.toRadians((165 + 180) % 360));
    private final Pose blueTarget = new Pose(30, 124,Math.toRadians((165 + 180) % 360));

    private static final double POSITION_TOLERANCE = 0.1;
    private static final double HEADING_TOLERANCE = 0.1;
    private static final double SERVO_TOGGLE_DELAY = 0.2;

    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;
    private RevBlinkinLedDriver blinkin;
    private Limelight3A limelight;
    private VoltageSensor voltageSensor;
    private MultipleTelemetry multiTelemetry;

    private boolean navigating = false;
    private boolean waitingForContinue = false;
    private Pose currentTarget = null;

    private boolean aPressed = false, bPressed = false;
    private boolean intakeFeedToggle = false, intakeFeedPressed = false;
    private boolean shooterToggle = false, shooterPressed = false;
    private boolean holdToggle = false, yPressed = false;
    private boolean leftBumperPressed = false, rightBumperPressed = false;
    private boolean sharePressed = false;
    private boolean shooterWasActive = false;

    private boolean defensiveDrivingEnabled = false;
    private Pose prevPose = null;
    private Pose intendedPose = null;
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double POSITION_HOLD_GAIN = 0.03;
    private static final double VELOCITY_GAIN = 0.01;

    private boolean limelightConnected = false;
    private boolean autoAimEnabled = false;
    private boolean artifactDetectionMode = false;
    private boolean hasValidTarget = false;
    private double targetTx = 0;
    private double targetTy = 0;
    private double targetArea = 0;
    private double currentDistanceToTarget = 0;

    private double currentVoltage = 12.5;
    private double adaptiveMaxPower = 0.4;
    private double manualRPMAdjustment = 0;

    private ElapsedTime servoToggleTimer = new ElapsedTime();

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new ShooterSubsystem(hardwareMap, multiTelemetry);
        intake = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(pushServo);
        servos.setHoldServo(holdServo);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = voltageSensor.getVoltage();

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        initializeLimelight();

        loopTimer.reset();

        multiTelemetry.addLine("═══════════════════════════════════");
        multiTelemetry.addLine("  LIMELIGHT 3A + ML TELEOP");
        multiTelemetry.addLine("  DECODE 2025-2026 Season");
        multiTelemetry.addLine("═══════════════════════════════════");
        multiTelemetry.addData("Limelight", limelightConnected ? "✓ CONNECTED" : "✗ NOT FOUND");
        multiTelemetry.addData("Battery Voltage", "%.2f V", currentVoltage);
        multiTelemetry.update();
    }

    private void initializeLimelight() {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            limelightConnected = true;
        } catch (Exception e) {
            limelightConnected = false;
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        currentVoltage = voltageSensor.getVoltage();
        updateLimelightData();

        double adaptiveRPM = calculateDistanceBasedRPM();
        adaptiveMaxPower = calculateAdaptiveSpeed();
        ShooterSubsystem.targetRPM = adaptiveRPM + manualRPMAdjustment;

        if (gamepad1.share && !sharePressed) {
            defensiveDrivingEnabled = !defensiveDrivingEnabled;
            sharePressed = true;
            if (!defensiveDrivingEnabled) intendedPose = null;
        } else if (!gamepad1.share) sharePressed = false;

        if (gamepad1.left_bumper && !leftBumperPressed) {
            autoAimEnabled = !autoAimEnabled;
            leftBumperPressed = true;
            if (autoAimEnabled && limelightConnected) limelight.pipelineSwitch(0);
        } else if (!gamepad1.left_bumper) leftBumperPressed = false;

        if (gamepad1.right_bumper && !rightBumperPressed) {
            artifactDetectionMode = !artifactDetectionMode;
            rightBumperPressed = true;
            if (limelightConnected) {
                int pipeline = artifactDetectionMode ? PURPLE_ARTIFACT_PIPELINE : GREEN_ARTIFACT_PIPELINE;
                limelight.pipelineSwitch(pipeline);
            }
        } else if (!gamepad1.right_bumper) rightBumperPressed = false;

        if (waitingForContinue) {
            follower.setTeleOpMovementVectors(0, 0, 0, false);
            follower.update();
            if (gamepad1.b && !bPressed) {
                waitingForContinue = false;
                currentTarget = null;
                follower.startTeleopDrive();
                bPressed = true;
            }
            multiTelemetry.addLine("Arrived at BLUE Target — Press B to continue");
            multiTelemetry.update();
            return;
        }

        if (gamepad1.a && !aPressed && !navigating) {
            startNavigation(blueTarget);
            aPressed = true;
        } else if (!gamepad1.a) aPressed = false;

        if (gamepad1.b && !bPressed && navigating) {
            cancelNavigation();
            bPressed = true;
        } else if (!gamepad1.b) bPressed = false;

        Pose pose = follower.getPose();
        if (pose == null) pose = startPose.copy();

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (prevPose == null) prevPose = pose.copy();

        if (navigating) {
            follower.update();
            if (arrivedAtTarget()) {
                navigating = false;
                waitingForContinue = true;
                follower.breakFollowing();
                intendedPose = null;
            }
        } else {
            // ═══ SWAPPED JOYSTICK MAPPING ═══
            double rotatedX = gamepad1.left_stick_y; // Forward/back → strafe
            double rotatedY = gamepad1.left_stick_x;  // Left/right → drive
            double rotation = -gamepad1.right_stick_x;

            if (autoAimEnabled && limelightConnected && hasValidTarget) {
                double rotationCorrection = -targetTx * AUTO_AIM_ROTATION_GAIN;
                rotationCorrection = Math.max(-AUTO_AIM_MAX_ROTATION, Math.min(AUTO_AIM_MAX_ROTATION, rotationCorrection));

                double horizontalError = targetTx + calculateHorizontalOffsetError();
                double strafeCorrection = -horizontalError * AUTO_AIM_STRAFE_GAIN;
                strafeCorrection = Math.max(-AUTO_AIM_MAX_STRAFE, Math.min(AUTO_AIM_MAX_STRAFE, strafeCorrection));

                rotation += rotationCorrection;
                rotatedX += strafeCorrection;

                rotation = Math.max(-1, Math.min(1, rotation));
                rotatedX = Math.max(-1, Math.min(1, rotatedX));
            }

            if (defensiveDrivingEnabled) {
                if (Math.abs(rotatedX) > 0.05 || Math.abs(rotatedY) > 0.05) {
                    intendedPose = pose.copy();
                } else if (intendedPose == null) intendedPose = pose.copy();

                double dx = intendedPose.getX() - pose.getX();
                double dy = intendedPose.getY() - pose.getY();
                double vx = (pose.getX() - prevPose.getX()) / (dt > 0 ? dt : 0.02);
                double vy = (pose.getY() - prevPose.getY()) / (dt > 0 ? dt : 0.02);

                double correctionX = dx * POSITION_HOLD_GAIN - vx * VELOCITY_GAIN;
                double correctionY = dy * POSITION_HOLD_GAIN - vy * VELOCITY_GAIN;

                double commandedX = rotatedX + correctionX;
                double commandedY = rotatedY + correctionY;

                commandedX = Math.max(-1, Math.min(1, commandedX));
                commandedY = Math.max(-1, Math.min(1, commandedY));

                follower.setTeleOpMovementVectors(commandedX, commandedY, rotation, false);
            } else {
                follower.setTeleOpMovementVectors(rotatedX, rotatedY, rotation, false);
            }

            follower.update();
            prevPose = pose.copy();
        }

        if (gamepad2.a && !intakeFeedPressed) {
            intakeFeedToggle = !intakeFeedToggle;
            intakeFeedPressed = true;
            if (intakeFeedToggle) {
                intake.start();
                transfer.start();
            } else {
                intake.stop();
                transfer.stop();
            }
        } else if (!gamepad2.a) intakeFeedPressed = false;

        if (gamepad2.right_trigger > 0.5 && !shooterPressed) {
            shooterToggle = !shooterToggle;
            shooterPressed = true;
        } else if (gamepad2.right_trigger <= 0.5) shooterPressed = false;

        if (shooterToggle) {
            shooter.update();
            if (!shooterWasActive) {
                shooterWasActive = true;
                gamepad2.rumble(1.0, 1.0, 100000);
            }
        } else {
            shooter.stop();
            if (shooterWasActive) {
                shooterWasActive = false;
                gamepad2.stopRumble();
            }
        }

        if (gamepad2.share) {
            intake.reverse();
            transfer.reverse();
        }

        if (gamepad2.y && !yPressed && servoToggleTimer.seconds() > SERVO_TOGGLE_DELAY) {
            holdToggle = !holdToggle;
            yPressed = true;
            servoToggleTimer.reset();
            if (holdToggle) servos.engageHold();
            else servos.retractHold();
        } else if (!gamepad2.y) yPressed = false;

        if (gamepad2.x) servos.engagePush();
        else servos.retractPush();

        if (gamepad2.left_bumper) manualRPMAdjustment -= 5;
        if (gamepad2.right_bumper) manualRPMAdjustment += 5;

        updateLEDs();
        displayTelemetry(pose);
    }

    private void updateLimelightData() {
        if (!ENABLE_LIMELIGHT || !limelightConnected) return;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            hasValidTarget = true;
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                targetTx = fiducial.getTargetXDegrees();
                targetTy = fiducial.getTargetYDegrees();
                targetArea = fiducial.getTargetArea();
                currentDistanceToTarget = calculateDistanceToTarget(targetTy, CLASSIFIER_HEIGHT_INCHES);
            } else if (result.getColorResults() != null && !result.getColorResults().isEmpty()) {
                LLResultTypes.ColorResult color = result.getColorResults().get(0);
                targetTx = color.getTargetXDegrees();
                targetTy = color.getTargetYDegrees();
                targetArea = color.getTargetArea();
            }
        } else hasValidTarget = false;
    }

    private double calculateDistanceToTarget(double ty, double targetHeight) {
        double angleToTarget = LIMELIGHT_ANGLE_DEGREES + ty;
        return Math.abs((targetHeight - LIMELIGHT_HEIGHT_INCHES) / Math.tan(Math.toRadians(angleToTarget)));
    }

    private double calculateHorizontalOffsetError() {
        if (currentDistanceToTarget <= 0) return 0;
        return Math.toDegrees(Math.atan2(LIMELIGHT_HORIZONTAL_OFFSET_INCHES, currentDistanceToTarget));
    }

    private double calculateDistanceBasedRPM() {
        if (!ENABLE_DISTANCE_SHOOTING || !hasValidTarget) return calculateAdaptiveRPM();
        double distanceDelta = currentDistanceToTarget - BASE_SHOOTING_DISTANCE;
        double distanceRPM = BASE_RPM + (distanceDelta * RPM_PER_INCH);
        return Math.max(MIN_SHOOTING_RPM, Math.min(MAX_SHOOTING_RPM, distanceRPM));
    }

    private double calculateAdaptiveRPM() {
        if (!ENABLE_ADAPTIVE_RPM) return ShooterSubsystem.targetRPM;
        double voltageOffset = currentVoltage - 12.5;
        return BASE_RPM + (voltageOffset * RPM_VOLTAGE_COEFFICIENT);
    }

    private double calculateAdaptiveSpeed() {
        if (!ENABLE_BATTERY_COMPENSATION) return 0.4;
        if (currentVoltage < MIN_VOLTAGE_THRESHOLD) {
            double speedReduction = (MIN_VOLTAGE_THRESHOLD - currentVoltage) * 0.1;
            return Math.max(0.2, 0.4 - speedReduction);
        }
        return 0.4;
    }

    private void updateLEDs() {
        boolean holdEngaged = servos.isHoldActive();
        double shooterRPM = shooter.getRPM();
        double targetRPM = ShooterSubsystem.targetRPM;
        boolean shooterReady = shooterRPM >= targetRPM * 0.95;

        if (shooterReady && !holdEngaged)
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        else if (shooterReady && holdEngaged)
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
        else if (holdEngaged)
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    private void startNavigation(Pose target) {
        currentTarget = target;
        navigating = true;
        intendedPose = null;

        follower.followPath(follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(follower.getPose()),
                        new com.pedropathing.pathgen.Point(target)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build());
    }

    private void cancelNavigation() {
        navigating = false;
        currentTarget = null;
        waitingForContinue = false;
        follower.breakFollowing();
        follower.startTeleopDrive();
    }

    private boolean arrivedAtTarget() {
        if (currentTarget == null) return false;
        Pose pose = follower.getPose();
        if (pose == null) pose = startPose.copy();
        double dx = currentTarget.getX() - pose.getX();
        double dy = currentTarget.getY() - pose.getY();
        double dist = Math.hypot(dx, dy);
        double headingError = Math.abs(currentTarget.getHeading() - pose.getHeading());
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        return dist < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE;
    }

    private void displayTelemetry(Pose pose) {
        multiTelemetry.addData("Drive Mode", navigating ? "AutoNav" : "Manual");
        multiTelemetry.addData("Defensive Driving", defensiveDrivingEnabled ? "ON" : "OFF");
        multiTelemetry.addData("Auto-Aim", autoAimEnabled ? "ACTIVE" : "OFF");
        multiTelemetry.addData("Artifact Mode", artifactDetectionMode ? "PURPLE" : "GREEN");
        if (hasValidTarget)
            multiTelemetry.addData("Distance", "%.1f in", currentDistanceToTarget);
        multiTelemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
        transfer.stop();
        shooter.stopShooter();
        servos.retractPush();
        servos.retractHold();
        if (limelightConnected) limelight.stop();
    }
}