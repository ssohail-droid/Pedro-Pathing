package pedroPathing.AutoClass.Dec6;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "FieldCentricBlueAuto", group = "Examples")
public class FieldCentricBlueAuto extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(40, 16, Math.toRadians(90));
    private final Pose targetPose = new Pose(24, 24, Math.toRadians(50));

    private DistanceSensor frontSensor;
    private DistanceSensor rightSensor;

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;
    private RevBlinkinLedDriver blinkin;

    private boolean xPressed = false;
    private boolean navigating = false;
    private boolean aligning = false;
    private boolean arrived = false;

    private boolean intakeFeedToggle = false, intakeFeedPressed = false;
    private boolean shooterToggle = false, shooterPressed = false;
    private boolean holdToggle = false, yPressed = false;
    private boolean shooterWasActive = false;

    private double manualRPMAdjustment = 0;
    private ElapsedTime servoToggleTimer = new ElapsedTime();

    public static double TARGET_FRONT_DIST = 4;
    public static double TARGET_RIGHT_DIST = 14;
    public static double ALIGN_TOLERANCE = 1.0;
    public static double ALIGN_SPEED = 0.1;

    public static double POSITION_TOLERANCE = 2.0;
    public static double HEADING_TOLERANCE_DEG = 5;
    public static long ALIGN_TIMEOUT_MS = 1700;

    private long alignStartTime;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        frontSensor = hardwareMap.get(DistanceSensor.class, "front_distance");
        rightSensor = hardwareMap.get(DistanceSensor.class, "left_distance");

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        intake = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);
        shooter = new ShooterSubsystem(hardwareMap, (MultipleTelemetry) telemetry);
        servos = new ServoSubsystem(pushServo);
        servos.setHoldServo(holdServo);

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        telemetry.addLine("Initialized. Press X to auto-align.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // === Gamepad 1: Start auto-align ===
        if (gamepad1.x && !xPressed && !navigating && !aligning) {
            xPressed = true;
            startAutoNavigation();
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        // === Gamepad 1: Cancel BOTH navigation + alignment ===
        if ((gamepad1.left_bumper || gamepad1.right_bumper) && (navigating || aligning)) {
            navigating = false;
            aligning = false;
            arrived = false;
            follower.breakFollowing();
            follower.startTeleopDrive();
            gamepad1.rumble(300);
            telemetry.addLine("Navigation/alignment canceled manually.");
        }

        // === AUTO NAVIGATION ===
        if (navigating && !arrived) {
            follower.update();

            if (hasArrivedAtTarget()) {
                navigating = false;
                arrived = true;

                follower.breakFollowing();
                follower.setPose(targetPose);
                follower.startTeleopDrive();

                aligning = true;
                alignStartTime = System.currentTimeMillis();
            }
        }

        // === SENSOR ALIGNMENT ===
        if (aligning) {
            if (System.currentTimeMillis() - alignStartTime > ALIGN_TIMEOUT_MS) {
                aligning = false;
                arrived = false;
                telemetry.addLine("!!! Alignment timed out !!!");
            } else {
                boolean alignedFront = false;
                boolean alignedRight = false;

                double frontDist = frontSensor.getDistance(DistanceUnit.INCH);
                double rightDist = rightSensor.getDistance(DistanceUnit.INCH);

                double yPower = 0;
                double xPower = 0;

                if (Math.abs(frontDist - TARGET_FRONT_DIST) > ALIGN_TOLERANCE)
                    yPower = (frontDist > TARGET_FRONT_DIST) ? ALIGN_SPEED : -ALIGN_SPEED;
                else alignedFront = true;

                if (Math.abs(rightDist - TARGET_RIGHT_DIST) > ALIGN_TOLERANCE)
                    xPower = (rightDist > TARGET_RIGHT_DIST) ? ALIGN_SPEED : -ALIGN_SPEED;
                else alignedRight = true;

                if (!(alignedFront && alignedRight)) {
                    follower.setTeleOpMovementVectors(xPower, yPower, 0, false);
                    follower.update();
                } else {
                    aligning = false;
                    arrived = false;
                    telemetry.addLine(">>> Auto-alignment complete. <<<");
                }
            }
        }

        // === DEFAULT DRIVE ===
        if (!navigating && !aligning) {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // === Gamepad 2 Controls ===
        handleGamepad2Controls();
        updateLEDs();

        // === Telemetry ===
        telemetry.addLine(navigating ? "Navigating to target..." :
                aligning ? "Auto-aligning with sensors..." :
                        "Manual drive mode");

        telemetry.addLine("=== Pose ===");
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading°", "%.1f", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("=== Sensors ===");
        telemetry.addData("Front (in)", "%.2f", frontSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right (in)", "%.2f", rightSensor.getDistance(DistanceUnit.INCH));

        telemetry.addLine("=== Alignment ===");
        telemetry.addData("Aligning", aligning);
        telemetry.addData("Timeout (ms)", ALIGN_TIMEOUT_MS);
        telemetry.addData("Time Elapsed", System.currentTimeMillis() - alignStartTime);

        telemetry.update();
    }

    private void handleGamepad2Controls() {
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
        } else if (!gamepad2.a) {
            intakeFeedPressed = false;
        }

        if (gamepad2.right_trigger > 0.5 && !shooterPressed) {
            shooterToggle = !shooterToggle;
            shooterPressed = true;
        } else if (gamepad2.right_trigger <= 0.5) {
            shooterPressed = false;
        }

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

        if (gamepad2.y && !yPressed && servoToggleTimer.seconds() > 0.2) {
            holdToggle = !holdToggle;
            yPressed = true;
            servoToggleTimer.reset();
            if (holdToggle) servos.engageHold();
            else servos.retractHold();
        } else if (!gamepad2.y) {
            yPressed = false;
        }

        if (gamepad2.x) servos.engagePush();
        else servos.retractPush();

        if (gamepad2.left_bumper) manualRPMAdjustment -= 5;
        if (gamepad2.right_bumper) manualRPMAdjustment += 5;

        ShooterSubsystem.targetRPM = 2400 + manualRPMAdjustment;
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

    private void startAutoNavigation() {
        navigating = true;
        arrived = false;

        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(follower.getPose()),
                                new Point(targetPose)
                        ))
                        .setLinearHeadingInterpolation(
                                follower.getPose().getHeading(),
                                targetPose.getHeading()
                        )
                        .build()
        );
    }

    private boolean hasArrivedAtTarget() {
        Pose current = follower.getPose();
        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distanceError = Math.hypot(dx, dy);
        double headingError = normalizeAngle(targetPose.getHeading() - current.getHeading());

        return distanceError < POSITION_TOLERANCE &&
                Math.abs(headingError) < Math.toRadians(HEADING_TOLERANCE_DEG);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}