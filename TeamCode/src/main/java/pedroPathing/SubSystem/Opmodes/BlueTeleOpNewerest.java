package pedroPathing.SubSystem.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;

/*
 * ═══════════════════════════════════════════════════════════════════════
 * BLUE TELEOP CONTROLS
 * ═══════════════════════════════════════════════════════════════════════
 *
 * GAMEPAD 1 (DRIVER):
 * - Left Stick: Drive (X/Y movement)
 * - Right Stick X: Rotation
 * - A Button: Navigate to Blue Auto Point (shooting position)
 * - B Button: Cancel navigation / Continue after arrival
 * - Share Button: Toggle Defensive Driving (position hold when sticks released)
 *
 * GAMEPAD 2 (OPERATOR):
 * - A Button: Toggle Intake + Feed (on/off)
 * - Right Trigger: Toggle Shooter (on/off with continuous rumble)
 * - Y Button: Toggle Hold Servo (hold artifacts in position)
 * - X Button (Hold): Push Servo Active (push artifacts into shooter)
 * - Share Button: Reverse Intake + Feed
 *
 * LED INDICATORS:
 * - STROBE RED: Hold engaged + Shooter at speed (ready to shoot)
 * - RED: Hold engaged, shooter warming up
 * - STROBE BLUE: Shooter at speed, hold not engaged
 * - BLUE: Default/idle state
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

@TeleOp(name = "BLUE TeleOp (Defensive Driving)", group = "Subsystems")
public class BlueTeleOpNewerest extends OpMode {

    private Follower follower;

    // Starting pose = End position of autonomous (shooting position)
    // This should match the final pose from your autonomous code
    private final Pose startPose = new Pose(60.5, 100, Math.toRadians(143));

    // Blue target = shooting position (same as start, for returning after manual drive)
    private final Pose blueTarget = new Pose(60.5, 100, Math.toRadians(143));

    private boolean navigating = false;
    private boolean waitingForContinue = false;
    private Pose currentTarget = null;

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;

    private boolean aPressed = false, bPressed = false;
    private boolean intakeFeedToggle = false, intakeFeedPressed = false;
    private boolean shooterToggle = false, shooterPressed = false;
    private boolean holdToggle = false;
    private boolean yPressed = false;

    private ElapsedTime servoToggleTimer = new ElapsedTime();
    private static final double SERVO_TOGGLE_DELAY = 0.2;

    private static final double POSITION_TOLERANCE = 0.1;
    private static final double HEADING_TOLERANCE = 0.1;

    private boolean shooterWasActive = false;

    private RevBlinkinLedDriver blinkin;

    // === Defensive Driving ===
    private boolean defensiveDrivingEnabled = false;
    private boolean sharePressed = false;
    private boolean reversePressed = false;
    private boolean reverseToggled = false;


    private Pose prevPose = null;
    private Pose intendedPose = null;
    private ElapsedTime loopTimer = new ElapsedTime();

    private static final double POSITION_HOLD_GAIN = 0.03;
    private static final double VELOCITY_GAIN = 0.01;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        // Initialize from autonomous end position (shooting position)
        follower.setStartingPose(startPose);

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        shooter = new ShooterSubsystem(hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        intake = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);

        // Initialize ServoSubsystem with push servo, then set hold servo
        servos = new ServoSubsystem(pushServo);
        servos.setHoldServo(holdServo);

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        loopTimer.reset();

        telemetry.addLine("Blue Alliance TeleOp (Right POV) — Shooter PIDF + Continuous Rumble Ready");
        telemetry.addLine("Starting from Autonomous End Position (Shooting Position)");
        telemetry.addLine("A: Go to Blue Auto Point");
        telemetry.addLine("B: Cancel or Continue");
        telemetry.addLine("Share: Toggle Defensive Driving");
        telemetry.addLine("Gamepad2 RT: Toggle Shooter (Continuous Rumble)");
        telemetry.addLine("Hold X: Push servo active");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // === Toggle Defensive Driving ===
        if (gamepad1.share && !sharePressed) {
            defensiveDrivingEnabled = !defensiveDrivingEnabled;
            sharePressed = true;
            if (!defensiveDrivingEnabled) {
                intendedPose = null;
            }
        } else if (!gamepad1.share) {
            sharePressed = false;
        }

        // ===== WAIT MODE =====
        if (waitingForContinue) {
            telemetry.addLine("Arrived at BLUE Target — Press B to continue");
            follower.setTeleOpMovementVectors(0, 0, 0, false);
            follower.update();

            if (!gamepad1.a) aPressed = false;
            if (!gamepad1.b) bPressed = false;

            if (gamepad1.b && !bPressed) {
                waitingForContinue = false;
                currentTarget = null;
                follower.startTeleopDrive();
                bPressed = true;
            }

            telemetry.addData("AutoNav State", "WaitingForContinue");
            telemetry.update();
            return;
        }

        // ===== AUTONAV CONTROLS =====
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
            double rotatedX = gamepad1.left_stick_x;
            double rotatedY = -gamepad1.left_stick_y;
            double rotation = -gamepad1.right_stick_x;

            if (defensiveDrivingEnabled) {
                if (Math.abs(rotatedX) > 0.05 || Math.abs(rotatedY) > 0.05) {
                    intendedPose = pose.copy();
                } else if (intendedPose == null) {
                    intendedPose = pose.copy();
                }

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

        // ═══════════════════════════════════════════════════════════════════════
        // GAMEPAD 2 CONTROLS (Operator)
        // ═══════════════════════════════════════════════════════════════════════

        // Toggle Intake + Feed (A button)
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

        // Toggle Shooter (Right Trigger with continuous rumble)
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

        // Reverse Intake + Feed (Share button)
        // =========================== START OF FIX ===========================
        if (!reversePressed && gamepad2.share) {
            reverseToggled = !reverseToggled;
            reversePressed = true;

            if (reverseToggled) {
                intake.reverse();
                transfer.reverse();
            } else {
                if (intakeFeedToggle) {
                    intake.start();
                    transfer.start();
                } else {
                    intake.stop();
                    transfer.stop();
                }
            }
        } else if (!gamepad2.share) {
            reversePressed = false;
        }
        // ============================ END OF FIX ============================

        // Toggle Hold Servo (Y button) - Holds artifacts in position
        if (gamepad2.y && !yPressed && servoToggleTimer.seconds() > SERVO_TOGGLE_DELAY) {
            holdToggle = !holdToggle;
            yPressed = true;
            servoToggleTimer.reset();

            if (holdToggle) servos.engageHold();
            else servos.retractHold();
        } else if (!gamepad2.y) yPressed = false;

        // Push Servo (X button - hold to push) - Pushes artifacts into shooter
        if (gamepad2.x) servos.engagePush();
        else servos.retractPush();

        // ═══════════════════════════════════════════════════════════════════════
        // LED CONTROL (Visual feedback for shooter/hold status)
        // ═══════════════════════════════════════════════════════════════════════
        boolean holdEngaged = servos.isHoldActive();
        double shooterRPM = shooter.getRPM();
        double targetRPM = ShooterSubsystem.targetRPM;

        boolean holdReady = holdEngaged && shooterRPM >= targetRPM * 0.95;

        if (holdEngaged && holdReady) {
            // Hold engaged + shooter at speed = READY TO SHOOT
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else if (holdEngaged) {
            // Hold engaged but shooter warming up
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (shooterToggle && shooterRPM >= targetRPM * 0.95) {
            // Shooter at speed, hold not engaged
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
        } else {
            // Default/idle
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // ═══════════════════════════════════════════════════════════════════════
        // TELEMETRY
        // ═══════════════════════════════════════════════════════════════════════
        telemetry.addLine(navigating ? "AutoNav: BLUE Target" : "Manual Drive (Right POV)");
        telemetry.addData("AutoNav State", navigating ? "Navigating" : (waitingForContinue ? "WaitingForContinue" : "Idle"));
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Shooter RPM", "%.1f", shooterRPM);
        telemetry.addData("Shooter Active", shooterToggle ? "ON" : "OFF");
        telemetry.addData("Hold Servo", holdEngaged ? "Engaged" : "Retracted");
        telemetry.addData("Hold Ready", holdReady ? "YES" : "NO");
        telemetry.addData("Push Servo", servos.isPushActive() ? "Engaged" : "Retracted");
        telemetry.addData("Defensive Driving", defensiveDrivingEnabled ? "ENABLED" : "DISABLED");

        if (defensiveDrivingEnabled && intendedPose != null) {
            telemetry.addData("Hold X Error", "%.2f", intendedPose.getX() - pose.getX());
            telemetry.addData("Hold Y Error", "%.2f", intendedPose.getY() - pose.getY());
        }

        telemetry.update();
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
}