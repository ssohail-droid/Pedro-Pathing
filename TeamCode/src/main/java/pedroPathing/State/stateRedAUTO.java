package pedroPathing.State;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "state Red AUTO(9ball)", group = "Main")
public class stateRedAUTO extends OpMode {

    // ──────────────── TIMERS ────────────────
    private Timer shootTimer = new Timer();
    private Timer settleTimer = new Timer();
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // ──────────────── HARDWARE ────────────────
    private DcMotorEx intake, shooter, upPipe, midPipe;
    private Servo adjustServo, gateServo;

    // ──────────────── TUNING ────────────────
    public static double INTAKE_POWER   = 1.0;
    public static double UP_PIPE_POWER  = 1.0;
    public static double MID_PIPE_POWER = 1.0;
    public static double TICKS_PER_REV  = 28.0;
    public static double RPM            = 2550;
    public static double RPM_TOL        = 75;
    public static double HOOD_POS       = 0.675;
    public static double GATE_OPEN      = 0.4;
    public static double GATE_CLOSED    = 0.1;
    public static double RPM_WAIT_MS    = 1000;
    public static double GATE_OPEN_MS   = 1000;
    public static double PIPE_DELAY_MS  = 100;
    public static double SETTLE_MS      = 200;

    // ──────────────── STATE FLAGS ────────────────
    private boolean prespin    = false;
    private boolean shooting   = false;
    private boolean gateOpened = false;
    private boolean pipesOn    = false;

    // ──────────────── POSES ────────────────
    private final Pose startPos              = new Pose(111, 135, Math.toRadians(0));
    private final Pose shootPos              = new Pose(102.08263837015359, 102.08263837015359, Math.toRadians(42));
    private final Pose intakeRowOnePos       = new Pose(95, 85, Math.toRadians(0));
    private final Pose intakePickUpRowOnePos = new Pose(128, 85, Math.toRadians(0));
    private final Pose intakeRowTwoPos       = new Pose(100, 62, Math.toRadians(0));
    private final Pose intakePickUpRowTwoPos = new Pose(137.1, 62, Math.toRadians(0));
    private final Pose leave                 = new Pose(96, 126.5, Math.toRadians(0));

    // ──────────────── PATHS ────────────────
    private PathChain moveOne, moveTwo, moveThree, moveFour;
    private PathChain moveFive, moveSix, moveSeven, moveLeave;

    private void buildPaths() {
        moveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowOnePos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowOnePos.getHeading())
                .build();

        moveThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowOnePos), new Point(intakePickUpRowOnePos)))
                .setLinearHeadingInterpolation(intakeRowOnePos.getHeading(), intakePickUpRowOnePos.getHeading())
                .build();

        moveFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePickUpRowOnePos), new Point(shootPos)))
                .setLinearHeadingInterpolation(intakePickUpRowOnePos.getHeading(), shootPos.getHeading())
                .build();

        moveFive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowTwoPos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowTwoPos.getHeading())
                .build();

        moveSix = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowTwoPos), new Point(intakePickUpRowTwoPos)))
                .setLinearHeadingInterpolation(intakeRowTwoPos.getHeading(), intakePickUpRowTwoPos.getHeading())
                .build();

        moveSeven = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowTwoPos),
                        new Point(100.929, 58.8972),
                        new Point(shootPos)
                ))
                .setLinearHeadingInterpolation(intakePickUpRowTwoPos.getHeading(), shootPos.getHeading())
                .build();

        moveLeave = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(leave)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), leave.getHeading())
                .build();
    }

    // ──────────────── SHOOT HELPERS ────────────────

    // Pre-spin during transit — just sets hood and velocity, no gate/timer
    private void startShooter(double rpm, double hoodPos) {
        prespin = true;
        adjustServo.setPosition(hoodPos);
        setShooterRPM(rpm);
    }

    // Call after settle — kicks off gate/pipe sequence
    private void beginShoot() {
        prespin    = false;
        shooting   = true;
        gateOpened = false;
        pipesOn    = false;
        gateServo.setPosition(GATE_CLOSED);
        shootTimer.resetTimer();
    }

    // Returns true when full shoot cycle is complete
    private boolean shootUpdate() {
        if (!shooting) return true;

        if (!gateOpened) {
            if (shooterAtSpeed() || shootTimer.getElapsedTimeSeconds() * 1000 >= RPM_WAIT_MS) {
                gateServo.setPosition(GATE_OPEN);
                gateOpened = true;
                shootTimer.resetTimer();
            }
        } else if (!pipesOn) {
            if (shootTimer.getElapsedTimeSeconds() * 1000 >= PIPE_DELAY_MS) {
                upPipe.setPower(UP_PIPE_POWER);
                midPipe.setPower(MID_PIPE_POWER);
                intake.setPower(INTAKE_POWER);
                pipesOn = true;
            }
        } else {
            if (shootTimer.getElapsedTimeSeconds() * 1000 >= GATE_OPEN_MS) {
                gateServo.setPosition(GATE_CLOSED);
                upPipe.setPower(0);
                midPipe.setPower(0);
                intake.setPower(0);
                shooting = false;
                return true;
            }
        }
        return false;
    }

    // ──────────────── PATH STATE MACHINE ────────────────
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:  // pre-spin and drive to shoot pos
                startShooter(RPM, HOOD_POS);
                follower.followPath(moveOne);
                setPathState(1);
                break;

            case 1:  // wait for arrival
                if (!follower.isBusy()) {
                    settleTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:  // settle
                if (settleTimer.getElapsedTimeSeconds() * 1000 >= SETTLE_MS) {
                    beginShoot();
                    setPathState(3);
                }
                break;

            case 3:  // shoot
                if (shootUpdate()) {
                    shooter.setPower(0);
                    follower.followPath(moveTwo);
                    setPathState(4);
                }
                break;

            case 4:  // shoot pos → row one approach, pre-spin for next shot
                if (!follower.isBusy()) {
                    startShooter(RPM, HOOD_POS);
                    follower.setMaxPower(0.6);
                    follower.followPath(moveThree);
                    setPathState(5);
                }
                break;

            case 5:  // sweep row one with intake
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    //intake.setPower(0);
                    follower.setMaxPower(1);
                    follower.followPath(moveFour);
                    setPathState(6);
                }
                break;

            case 6:  // return to shoot pos
                if (!follower.isBusy()) {
                    settleTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:  // settle
                if (settleTimer.getElapsedTimeSeconds() * 1000 >= SETTLE_MS) {
                    beginShoot();
                    setPathState(8);
                }
                break;

            case 8:  // shoot
                if (shootUpdate()) {
                    shooter.setPower(0);
                    follower.followPath(moveFive);
                    setPathState(9);
                }
                break;

            case 9:  // shoot pos → row two approach, pre-spin for next shot
                if (!follower.isBusy()) {
                    startShooter(RPM, HOOD_POS);
                    follower.setMaxPower(0.6);
                    follower.followPath(moveSix);
                    setPathState(10);
                }
                break;

            case 10:  // sweep row two with intake
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    //intake.setPower(0);
                    follower.setMaxPower(1);
                    follower.followPath(moveSeven);
                    setPathState(11);
                }
                break;

            case 11:  // return to shoot pos
                if (!follower.isBusy()) {
                    settleTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:  // settle
                if (settleTimer.getElapsedTimeSeconds() * 1000 >= SETTLE_MS) {
                    beginShoot();
                    setPathState(13);
                }
                break;

            case 13:  // shoot
                if (shootUpdate()) {
                    shooter.setPower(0);
                    follower.followPath(moveLeave);
                    setPathState(14);
                }
                break;

            case 14:  // park and stop everything
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    gateServo.setPosition(GATE_CLOSED);
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }

    // ──────────────── LIFECYCLE ────────────────
    @Override
    public void init() {
        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        buildPaths();

        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        upPipe  = hardwareMap.get(DcMotorEx.class, "upPipe");
        midPipe = hardwareMap.get(DcMotorEx.class, "midPipe");
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(70, 0, 8, 13.5)
        );

        upPipe.setDirection(DcMotorSimple.Direction.REVERSE);
        midPipe.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        adjustServo = hardwareMap.get(Servo.class, "hoodServo");
        gateServo   = hardwareMap.get(Servo.class, "stop");

        adjustServo.setPosition(HOOD_POS);
        gateServo.setPosition(GATE_CLOSED);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State",  pathState);
        telemetry.addData("Prespin",     prespin);
        telemetry.addData("Shooting",    shooting);
        telemetry.addData("Shooter RPM", getShooterRPM());
        telemetry.addData("At Speed",    shooterAtSpeed());
        telemetry.addData("Busy",        follower.isBusy());
        telemetry.update();
    }

    // ──────────────── SHOOTER HELPERS ────────────────
    private void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * TICKS_PER_REV) / 60.0);
    }

    private double getShooterRPM() {
        return shooter.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    private boolean shooterAtSpeed() {
        return Math.abs(getShooterRPM() - RPM) < RPM_TOL;
    }
}