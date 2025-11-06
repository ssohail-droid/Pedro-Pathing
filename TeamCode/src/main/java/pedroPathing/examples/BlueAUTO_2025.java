package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "BlueAUTO 2025 DECODE", group = "SeasonAutos")
public class BlueAUTO_2025 extends OpMode {

    public static double adaptiveMaxPower = 0.5;
    public static int totalPreloadShots = 3;
    public static int totalCycleShots = 2;

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;

    private VoltageSensor voltageSensor;
    private double currentVoltage = 12.5;
    private MultipleTelemetry multiTelemetry;

    private int shotStep = 0;
    private int totalShots = 3;
    private int currentShot = 0;
    private boolean rpmDipped = false;
    private long stepStartTime = 0;

    private final Pose startPose         = new Pose(22, 122.5, Math.toRadians((143 + 180) % 360));
    private final Pose shootPose         = new Pose(52, 102.5, Math.toRadians((143 + 180) % 360));
    private final Pose preIntakePose     = new Pose(52, 84,   Math.toRadians((0 + 180) % 360));
    private final Pose intakeMovePose    = new Pose(25, 84,   Math.toRadians((0 + 180) % 360));
    private final Pose intakeReturnPose  = new Pose(52, 84,   Math.toRadians((0 + 180) % 360));
    private final Pose intakeToShootPose = new Pose(52, 102.5,Math.toRadians((143 + 180) % 360));

    private PathChain toShootFromStart, toPreIntake, intakeOnMove, intakeOffReturn, toShootReturn;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor feedMotor   = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo pushServo     = hardwareMap.get(Servo.class, "push_servo");
        Servo holdServo     = hardwareMap.get(Servo.class, "hold_servo");

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter  = new ShooterSubsystem(hardwareMap, multiTelemetry);
        intake   = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);
        servos   = new ServoSubsystem(pushServo);
        servos.setHoldServo(holdServo);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = voltageSensor.getVoltage();

        pathTimer = new Timer();
        multiTelemetry.addLine("✅ BlueAUTO_2025 Ready");
        multiTelemetry.addData("Start Pose", startPose);
        multiTelemetry.update();
    }

    private void buildPaths() {
        toShootFromStart = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(shootPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        toPreIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPose), new Point(preIntakePose)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preIntakePose.getHeading())
                .build();

        intakeOnMove = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preIntakePose), new Point(intakeMovePose)))
                .setLinearHeadingInterpolation(preIntakePose.getHeading(), intakeMovePose.getHeading())
                .build();

        intakeOffReturn = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeMovePose), new Point(intakeReturnPose)))
                .setLinearHeadingInterpolation(intakeMovePose.getHeading(), intakeReturnPose.getHeading())
                .build();

        toShootReturn = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeReturnPose), new Point(intakeToShootPose)))
                .setLinearHeadingInterpolation(intakeReturnPose.getHeading(), intakeToShootPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        setPathState(0);
        follower.followPath(toShootFromStart);
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        shotStep = 0;
        currentShot = 0;
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        currentVoltage = voltageSensor.getVoltage();
        autoUpdate();

        multiTelemetry.addData("Path", pathState);
        multiTelemetry.addData("Shots", "%d/%d", currentShot, totalShots);
        multiTelemetry.addData("RPM", shooter.getRPM());
        multiTelemetry.addData("ShotStep", shotStep);
        multiTelemetry.update();
    }

    private void autoUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    totalShots = totalPreloadShots;
                    setPathState(1);
                }
                break;

            case 1:
                if (shotStep == 0 && currentShot == 0) {
                    shooter.startShooter();
                }
                if (handleShootingPhase()) {
                    follower.followPath(toPreIntake);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intake.start();
                    transfer.start();
                    follower.followPath(intakeOnMove);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(intakeOffReturn);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    intake.stop();
                    transfer.stop();
                    follower.followPath(toShootReturn);
                    totalShots = totalCycleShots;
                    setPathState(5);
                }
                break;

            case 5:
                if (handleShootingPhase()) {
                    shooter.stopShooter();
                    setPathState(-1);
                }
                break;
        }
    }

    private boolean handleShootingPhase() {
        long now = System.currentTimeMillis();
        double currentRPM = shooter.getRPM();

        switch (shotStep) {
            case 0:
                // Step 0: engage hold (keep artifact secure)
                servos.engageHold();
                intake.start();
                transfer.start();
                rpmDipped = false;
                stepStartTime = now;
                shotStep = 1;
                break;

            case 1:
                // Retract (open) hold when shooter is up to speed
                if (currentRPM >= ShooterSubsystem.targetRPM * 0.97 && !rpmDipped) {
                    servos.retractHold(); // ✅ open gate to release when ready
                }

                // Detect RPM dip = shot fired
                if (currentRPM < ShooterSubsystem.targetRPM * 0.90) {
                    rpmDipped = true;
                    intake.stop();
                    transfer.stop();
                    servos.engageHold(); // ✅ close again after firing
                    stepStartTime = now;
                    shotStep = 4;
                } else if (now - stepStartTime > 5000) {
                    // Fallback push
                    servos.engagePush();
                    stepStartTime = now;
                    shotStep = 2;
                }
                break;

            case 2:
                if (now - stepStartTime >= 1000) {
                    servos.retractPush();
                    servos.engageHold();
                    intake.stop();
                    transfer.stop();
                    stepStartTime = now;
                    shotStep = 4;
                }
                break;

            case 4:
                if (now - stepStartTime >= 500) {
                    currentShot++;
                    if (currentShot < totalShots) {
                        shotStep = 0;
                    } else {
                        return true;
                    }
                }
                break;
        }
        return false;
    }

    @Override
    public void stop() {
        intake.stop();
        transfer.stop();
        shooter.stopShooter();
        servos.emergencyStop();
    }
}
