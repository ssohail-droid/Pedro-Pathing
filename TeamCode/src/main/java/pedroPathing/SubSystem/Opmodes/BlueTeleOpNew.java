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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver; // ✅ Blinkin import

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;

@TeleOp(name = "BLUE TeleOp (Right POV, Shooter PIDF + Continuous Rumble)", group = "Subsystems")
public class BlueTeleOpNew extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(134.3, 34, 0);
    private final Pose blueTarget = new Pose(108, 36, Math.toRadians(222.5));

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
    private boolean servoToggle = false;
    private boolean yPressed = false;
    private boolean xPressed = false;

    private ElapsedTime servoToggleTimer = new ElapsedTime();
    private static final double SERVO_TOGGLE_DELAY = 0.2;

    private static final double POSITION_TOLERANCE = 0.1;
    private static final double HEADING_TOLERANCE = 0.1;

    private boolean shooterWasActive = false;

    private RevBlinkinLedDriver blinkin; // ✅ Blinkin field

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        shooter = new ShooterSubsystem(hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        intake = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(holdServo);
        servos.setPushServo2(pushServo);

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"); // ✅ Initialize Blinkin

        telemetry.addLine("Blue Alliance TeleOp (Right POV) — Shooter PIDF + Continuous Rumble Ready");
        telemetry.addLine("A: Go to Blue Auto Point");
        telemetry.addLine("B: Cancel or Continue");
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
        // ===== WAIT MODE =====
        if (waitingForContinue) {
            telemetry.addLine("Arrived at BLUE Target — Press B to continue");
            follower.setTeleOpMovementVectors(0, 0, 0, false);
            follower.update();

            if (gamepad1.b && !bPressed) {
                waitingForContinue = false;
                currentTarget = null;
                follower.startTeleopDrive();
                bPressed = true;
            } else if (!gamepad1.b) bPressed = false;
            telemetry.update();
            return;
        }

        // ===== AUTONAV =====
        if (gamepad1.a && !aPressed && !navigating) {
            startNavigation(blueTarget);
            aPressed = true;
        } else if (!gamepad1.a) aPressed = false;

        if (gamepad1.b && !bPressed && navigating) {
            cancelNavigation();
            bPressed = true;
        } else if (!gamepad1.b) bPressed = false;

        Pose pose = follower.getPose();
        if (pose == null) pose = startPose;

        if (navigating) {
            follower.update();
            if (arrivedAtTarget()) {
                navigating = false;
                waitingForContinue = true;
                follower.breakFollowing();
            }
        } else {
            double rotatedX = gamepad1.left_stick_x;
            double rotatedY = -gamepad1.left_stick_y;
            follower.setTeleOpMovementVectors(rotatedX, rotatedY, -gamepad1.right_stick_x, false);
            follower.update();
        }

        // ===== GAMEPAD 2 CONTROLS =====

        // Intake + Transfer toggle
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

        // Shooter toggle (PIDF velocity control + continuous rumble)
        if (gamepad2.right_trigger > 0.5 && !shooterPressed) {
            shooterToggle = !shooterToggle;
            shooterPressed = true;
        } else if (gamepad2.right_trigger <= 0.5) shooterPressed = false;

        if (shooterToggle) {
            shooter.update();

            if (!shooterWasActive) {
                shooterWasActive = true;
                // Start continuous rumble when shooter turns on
                gamepad2.rumble(1.0, 1.0, 100000); // 100 seconds (effectively continuous)
            }
        } else {
            shooter.stop();
            if (shooterWasActive) {
                shooterWasActive = false;
                // Stop rumble when shooter turns off
                gamepad2.stopRumble();
            }
        }

        if (gamepad2.share) {
            intake.reverse();
            transfer.reverse();
        }

        // Hold servo toggle (Y button)
        if (gamepad2.y && !yPressed && servoToggleTimer.seconds() > SERVO_TOGGLE_DELAY) {
            servoToggle = !servoToggle;
            yPressed = true;
            servoToggleTimer.reset();

            if (servoToggle) servos.engagePush();
            else servos.retractPush();
        } else if (!gamepad2.y) {
            yPressed = false;
        }

        // Push servo HOLD control (hold X to extend, release to retract)
        if (gamepad2.x) {
            servos.engagePush2();
        } else {
            servos.retractPush2();
        }

        // ===== LED CONTROL (Blinkin) =====
        boolean holdEngaged = servos.isPushActive();
        double shooterRPM = shooter.getRPM();
        double targetRPM = ShooterSubsystem.targetRPM;

        if (holdEngaged) {
            // 🔴 Hold gate is closed
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (shooterToggle && shooterRPM >= targetRPM * 0.95) {
            // ✳️ Shooter is ready
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
        } else {
            // 🟢 Shooter not ready
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // ===== TELEMETRY =====
        telemetry.addLine(navigating ? "AutoNav: BLUE Target" : "Manual Drive (Right POV)");
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Shooter RPM", "%.1f", shooterRPM);
        telemetry.addData("Shooter Active", shooterToggle ? "ON" : "OFF");
        telemetry.addData("Hold Servo", holdEngaged ? "Engaged" : "Retracted");
        telemetry.addData("Push Servo", servos.isPush2Active() ? "Engaged" : "Retracted");

        // ✅ LED Status Telemetry
        telemetry.addData("LED Pattern",
                holdEngaged ? "RED (Hold Closed)" :
                        (shooterToggle && shooterRPM >= targetRPM * 0.95) ? "STROBE GREEN (Shooter Ready)" :
                                "GREEN (Shooter Not Ready)");

        telemetry.update();
    }

    private void startNavigation(Pose target) {
        currentTarget = target;
        navigating = true;
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
        if (pose == null) pose = startPose;
        double dx = currentTarget.getX() - pose.getX();
        double dy = currentTarget.getY() - pose.getY();
        double dist = Math.hypot(dx, dy);
        double headingError = Math.abs(currentTarget.getHeading() - pose.getHeading());
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        return dist < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE;
    }
}
