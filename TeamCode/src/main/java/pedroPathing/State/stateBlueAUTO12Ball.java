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

@Autonomous(name = "state blue AUTO(12ball)", group = "Main")
public class stateBlueAUTO12Ball extends OpMode {

    // ──────────────── TIMERS ────────────────
    private Timer shootTimer  = new Timer();
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
    private final Pose startPos = new Pose(32, 135, Math.toRadians(180));
    private final Pose shootPos = new Pose(44.94409937888198, 99.05590062111803, Math.toRadians(137));


    private final Pose intakeRowOnePos = new Pose(52, 88, Math.toRadians(180));
    /// ///////////////////////////////////////////////^^from ball
    private final Pose intakePickUpRowOnePos = new Pose(16, 88, Math.toRadians(180));
    /// /////////////////////////////////////////////////////^^from gg

    private final Pose intakeRowTwoPos = new Pose(52, 63, Math.toRadians(180));
    /// ///////////////////////////////////////////////^^from ball
    private final Pose intakePickUpRowTwoPos = new Pose(7 , 63, Math.toRadians(180));
    private final Pose intakeRowThreePos       = new Pose(44.612850082372326, 35.5189456342669, Math.toRadians(180));
    private final Pose intakePickUpRowThreePos = new Pose(16.382207578253713, 35.5189456342669, Math.toRadians(180));
    private final Pose leave = new Pose(49.357495881383855, 123.7693574958814, Math.toRadians(180));

    // ──────────────── PATHS ────────────────
    private PathChain moveOne, moveTwo, moveThree, moveFour;
    private PathChain moveFive, moveSix, moveSeven;
    private PathChain moveEight, moveNine, moveTen;
    private PathChain moveLeave;

    private void buildPaths() {

        // start → shoot pos
        moveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

        // shoot pos → row one start
        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowOnePos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowOnePos.getHeading())
                .build();

        // row one sweep
        moveThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowOnePos), new Point(intakePickUpRowOnePos)))
                .setLinearHeadingInterpolation(intakeRowOnePos.getHeading(), intakePickUpRowOnePos.getHeading())
                .build();

        // row one end → shoot pos
        moveFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePickUpRowOnePos), new Point(shootPos)))
                .setLinearHeadingInterpolation(intakePickUpRowOnePos.getHeading(), shootPos.getHeading())
                .build();

        // shoot pos → row two start
        moveFive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowTwoPos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowTwoPos.getHeading())
                .build();

        // row two sweep
        moveSix = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowTwoPos), new Point(intakePickUpRowTwoPos)))
                .setLinearHeadingInterpolation(intakeRowTwoPos.getHeading(), intakePickUpRowTwoPos.getHeading())
                .build();

        // row two end → shoot pos (curve to avoid field elements)
        moveSeven = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowTwoPos),
                        new Point(100.929, 58.8972),
                        new Point(shootPos)
                ))
                .setLinearHeadingInterpolation(intakePickUpRowTwoPos.getHeading(), shootPos.getHeading())
                .build();

        // shoot pos → row three start
        moveEight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowThreePos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowThreePos.getHeading())
                .build();

        // row three sweep
        moveNine = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowThreePos), new Point(intakePickUpRowThreePos)))
                .setLinearHeadingInterpolation(intakeRowThreePos.getHeading(), intakePickUpRowThreePos.getHeading())
                .build();

        // row three end → shoot pos
        moveTen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePickUpRowThreePos), new Point(shootPos)))
                .setLinearHeadingInterpolation(intakePickUpRowThreePos.getHeading(), shootPos.getHeading())
                .build();

        // shoot pos → park
        moveLeave = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(leave)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), leave.getHeading())
                .build();
    }

    // ──────────────── SHOOT HELPERS ────────────────

    private void startShooter(double rpm, double hoodPos) {
        prespin = true;
        adjustServo.setPosition(hoodPos);
        setShooterRPM(rpm);
    }

    private void beginShoot() {
        prespin    = false;
        shooting   = true;
        gateOpened = false;
        pipesOn    = false;
        gateServo.setPosition(GATE_CLOSED);
        shootTimer.resetTimer();
    }

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
                pipesOn = true;
            }
        } else {
            if (shootTimer.getElapsedTimeSeconds() * 1000 >= GATE_OPEN_MS) {
                gateServo.setPosition(GATE_CLOSED);
                upPipe.setPower(0);
                midPipe.setPower(0);
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
                intake.setPower(INTAKE_POWER);
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
                    // intake.setPower(0);
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
                    follower.followPath(moveEight);
                    setPathState(14);
                }
                break;

            case 14:  // shoot pos → row three approach, pre-spin for next shot
                if (!follower.isBusy()) {
                    startShooter(RPM, HOOD_POS);
                    follower.setMaxPower(0.6);
                    follower.followPath(moveNine);
                    setPathState(15);
                }
                break;

            case 15:  // sweep row three with intake
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    // intake.setPower(0);
                    follower.setMaxPower(1);
                    follower.followPath(moveTen);
                    setPathState(16);
                }
                break;

            case 16:  // return to shoot pos
                if (!follower.isBusy()) {
                    settleTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:  // settle
                if (settleTimer.getElapsedTimeSeconds() * 1000 >= SETTLE_MS) {
                    beginShoot();
                    setPathState(18);
                }
                break;

            case 18:  // shoot
                if (shootUpdate()) {
                    shooter.setPower(0);
                    follower.followPath(moveLeave);
                    setPathState(19);
                }
                break;

            case 19:  // park and stop everything
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