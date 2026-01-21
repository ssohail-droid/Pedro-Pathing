package pedroPathing.Jan10.TELEOP.TESTOP;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Bluee Auto", group = "Main")
public class BlueAutoTeest extends OpMode {

    /* ================= DRIVE ================= */
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPos = new Pose(32, 135, Math.toRadians(180));
    private final Pose shootPos = new Pose(41, 111.05, Math.toRadians(140));

    private final Pose intakeRowOnePos = new Pose(52, 78, Math.toRadians(0));
    private final Pose intakePickUpRowOnePos = new Pose(22, 78, Math.toRadians(0));

    private final Pose intakeRowTwoPos = new Pose(52, 55, Math.toRadians(0));
    private final Pose intakePickUpRowTwoPos = new Pose(24.5 , 55, Math.toRadians(0));

    private PathChain moveOne, moveTwo, moveThree, moveFour;
    private PathChain moveFive, moveSix, moveSeven;

    /* ================= HARDWARE ================= */
    private DcMotorEx intake, shooter;
    private CRServo crLeft, crRight;
    private Servo spinServo, kickServo, adjustServo;
    private DistanceSensor distanceSensor;

    /* ================= PWM ================= */
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    /* ================= INTAKE ================= */
    public static double INTAKE_POWER = 0.8;
    public static double CR_INTAKE_POWER = 1.0;
    public static double DISTANCE_CM = 2.0;

    public static double INTAKE_0 = 0.145;
    public static double INTAKE_1 = 0.41;
    public static double INTAKE_2 = 0.7;

    private int ballCount = 0;
    private boolean lastDetected = false;
    private final ElapsedTime detectTimer = new ElapsedTime();

    /* ================= SHOOTER ================= */
    public static double TICKS_PER_REV = 28.0;
    public static double RPM = 2390;
    public static double RPM_TOL = 75;
    public static double HOOD_POS = 0.75;

    public static double SHOOT_A = 0.56;
    public static double SHOOT_B = 0.28;
    public static double SHOOT_C = 0.00;

    /* ================= KICKER ================= */
    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double KICK_MS = 500;
    public static double POST_MS = 500;

    /* ================= SHOOT ROUNDS ================= */
    private enum ShootRound { PRELOAD, ROW1, ROW2 }
    private ShootRound activeRound = ShootRound.PRELOAD;

    private enum ShootState { START, MOVE, DAMP, SETTLE, KICK, POST }
    private ShootState shootState = ShootState.START;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private int shootIndex = 0;
    private char currentBall = 'A';
    private boolean prespin = false;

    /* ================= PER-ROUND CONFIG ================= */

    // ---------- PRELOAD ----------
    public static String PRE_SEQ = "ABC";
    public static double PRE_MOVE = 0.18;
    public static double PRE_DAMP_A = 0.30, PRE_DAMP_B = 0.30, PRE_DAMP_C = 0.30;
    public static double PRE_SET_A = 0.35, PRE_SET_B = 0.35, PRE_SET_C = 0.35;
    public static double PRE_CR_L = 1.0, PRE_CR_R = -1.0;

    // ---------- ROW 1 ----------
    public static String R1_SEQ = "ACB";
    public static double R1_MOVE = 0.22;
    public static double R1_DAMP_A = 0.45, R1_DAMP_B = 0.30, R1_DAMP_C = 0.30;
    public static double R1_SET_A = 0.45, R1_SET_B = 0.35, R1_SET_C = 0.35;
    public static double R1_CR_L = 1.0, R1_CR_R = -1.0;

    // ---------- ROW 2 ----------
    public static String R2_SEQ = "ABC";
    public static double R2_MOVE = 0.20;
    public static double R2_DAMP_A = 0.30, R2_DAMP_B = 0.30, R2_DAMP_C = 0.30;
    public static double R2_SET_A = 0.35, R2_SET_B = 0.35, R2_SET_C = 0.35;
    public static double R2_CR_L = 1.0, R2_CR_R = -1.0;

    /* ================= PATHS ================= */
    public void buildPaths() {

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
                .addPath(new BezierLine(new Point(intakePickUpRowTwoPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(intakePickUpRowTwoPos.getHeading(), shootPos.getHeading())
                .build();
    }

    /* ================= AUTO FSM ================= */
    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                prespin = true;
                setShooterRPM(RPM);
                adjustServo.setPosition(HOOD_POS);
                follower.followPath(moveOne);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(ShootRound.PRELOAD);
                    setPathState(2);
                }
                break;

            case 2:
                updateMechanisms();
                if (mode == Mode.IDLE) {
                    ballCount = 0;
                    follower.followPath(moveTwo);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    prespin = true;
                    setShooterRPM(RPM);
                    follower.setMaxPower(0.2);
                    follower.followPath(moveThree);
                    setPathState(4);
                }
                break;

            case 4:
                prespin = true;
                setShooterRPM(RPM);
                runIntake();
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    stopIntake();
                    follower.followPath(moveFour);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(ShootRound.ROW1);
                    setPathState(6);
                }
                break;

            case 6:
                updateMechanisms();
                if (mode == Mode.IDLE) {
                    ballCount = 0;
                    follower.followPath(moveFive);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    prespin = true;
                    setShooterRPM(RPM);
                    follower.setMaxPower(0.2);
                    follower.followPath(moveSix);
                    setPathState(8);
                }
                break;

            case 8:
                prespin = true;
                setShooterRPM(RPM);
                runIntake();
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    stopIntake();
                    follower.followPath(moveSeven);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(ShootRound.ROW2);
                    setPathState(10);
                }
                break;

            case 10:
                updateMechanisms();
                if (mode == Mode.IDLE) setPathState(-1);
                break;
        }
    }

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }

    /* ================= INIT ================= */
    @Override
    public void init() {
        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        buildPaths();

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(70, 0, 8, 13.5)
        );

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo = hardwareMap.get(Servo.class, "spin");
        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        adjustServo = hardwareMap.get(Servo.class, "adjust_servo");

        if (spinServo instanceof PwmControl)
            ((PwmControl) spinServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        if (adjustServo instanceof PwmControl)
            ((PwmControl) adjustServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        kickServo.setPosition(KICK_IDLE);
        spinServo.setPosition(SHOOT_A);
        adjustServo.setPosition(HOOD_POS);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    /* ================= SHOOT LOGIC ================= */

    private enum Mode { IDLE, SHOOT }
    private Mode mode = Mode.IDLE;

    private void startShoot(ShootRound round) {
        activeRound = round;
        shootIndex = 0;
        shootState = ShootState.START;
        shootTimer.reset();
        setShooterRPM(RPM);
        mode = Mode.SHOOT;
    }

    private void updateMechanisms() {
        if (mode == Mode.SHOOT) runShoot();
        else hold();
    }

    private void runShoot() {

        String seq = getSeq();
        if (shootIndex >= seq.length()) {
            setCR(0, 0);
            setShooterRPM(0);
            spinServo.setPosition(INTAKE_0);
            mode = Mode.IDLE;
            return;
        }

        switch (shootState) {

            case START:
                currentBall = seq.charAt(shootIndex);
                spinServo.setPosition(getSpinFor(currentBall));
                shootTimer.reset();
                shootState = ShootState.MOVE;
                break;

            case MOVE:
                if (shootTimer.seconds() >= getMove()) {
                    setCR(0, 0);
                    shootTimer.reset();
                    shootState = ShootState.DAMP;
                }
                break;

            case DAMP:
                if (shootTimer.seconds() >= getDamp(currentBall)) {
                    setCR(getCR_L(), getCR_R());
                    shootTimer.reset();
                    shootState = ShootState.SETTLE;
                }
                break;

            case SETTLE:
                if (shootTimer.seconds() >= getSettle(currentBall) && shooterAtSpeed()) {
                    kickServo.setPosition(KICK_ACTIVE);
                    shootTimer.reset();
                    shootState = ShootState.KICK;
                }
                break;

            case KICK:
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    shootTimer.reset();
                    shootState = ShootState.POST;
                }
                break;

            case POST:
                if (shootTimer.milliseconds() >= POST_MS) {
                    shootIndex++;
                    shootState = ShootState.START;
                }
                break;
        }
    }

    /* ================= INTAKE ================= */

    private void runIntake() {
        if (ballCount >= 3) {
            stopIntake();
            return;
        }

        intake.setPower(INTAKE_POWER);
        setCR(CR_INTAKE_POWER, CR_INTAKE_POWER);

        boolean detected = distanceSensor.getDistance(DistanceUnit.CM) <= DISTANCE_CM;
        if (detected && !lastDetected && detectTimer.seconds() > 0.4) {
            ballCount++;
            detectTimer.reset();
        }
        lastDetected = detected;

        spinServo.setPosition(ballCount == 0 ? INTAKE_0 :
                ballCount == 1 ? INTAKE_1 : INTAKE_2);
    }

    private void stopIntake() {
        intake.setPower(0);
        setCR(0, 0);
    }

    private void hold() {
        stopIntake();
        if (!prespin) setShooterRPM(0);
        kickServo.setPosition(KICK_IDLE);
    }

    /* ================= HELPERS ================= */

    private void setCR(double l, double r) {
        crLeft.setPower(l);
        crRight.setPower(r);
    }

    private void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * TICKS_PER_REV) / 60.0);
    }

    private boolean shooterAtSpeed() {
        double rpm = shooter.getVelocity() * 60.0 / TICKS_PER_REV;
        return Math.abs(rpm - RPM) < RPM_TOL;
    }

    private double getSpinFor(char c) {
        return c == 'A' ? SHOOT_A : c == 'B' ? SHOOT_B : SHOOT_C;
    }

    private String getSeq() {
        return activeRound == ShootRound.PRELOAD ? PRE_SEQ :
                activeRound == ShootRound.ROW1 ? R1_SEQ :
                        R2_SEQ;
    }

    private double getMove() {
        return activeRound == ShootRound.PRELOAD ? PRE_MOVE :
                activeRound == ShootRound.ROW1 ? R1_MOVE :
                        R2_MOVE;
    }

    private double getCR_L() {
        return activeRound == ShootRound.PRELOAD ? PRE_CR_L :
                activeRound == ShootRound.ROW1 ? R1_CR_L :
                        R2_CR_L;
    }

    private double getCR_R() {
        return activeRound == ShootRound.PRELOAD ? PRE_CR_R :
                activeRound == ShootRound.ROW1 ? R1_CR_R :
                        R2_CR_R;
    }

    private double getDamp(char c) {
        if (activeRound == ShootRound.PRELOAD)
            return c=='A'?PRE_DAMP_A:c=='B'?PRE_DAMP_B:PRE_DAMP_C;
        if (activeRound == ShootRound.ROW1)
            return c=='A'?R1_DAMP_A:c=='B'?R1_DAMP_B:R1_DAMP_C;
        return c=='A'?R2_DAMP_A:c=='B'?R2_DAMP_B:R2_DAMP_C;
    }

    private double getSettle(char c) {
        if (activeRound == ShootRound.PRELOAD)
            return c=='A'?PRE_SET_A:c=='B'?PRE_SET_B:PRE_SET_C;
        if (activeRound == ShootRound.ROW1)
            return c=='A'?R1_SET_A:c=='B'?R1_SET_B:R1_SET_C;
        return c=='A'?R2_SET_A:c=='B'?R2_SET_B:R2_SET_C;
    }
}