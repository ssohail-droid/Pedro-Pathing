package pedroPathing.Jan10.TELEOP.DASAUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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

@Autonomous(name = "RedAutoJointV3_FULL", group = "Main")
public class RedAutoJointV3_FULL extends OpMode {

    /* ================= DRIVE ================= */
    private Follower follower;
    private Timer pathTimer;

    /* ================= SPEED CONTROL ================= */
    public static double SPEED_TO_SHOOT       = 1.00;

    public static double SPEED_TO_ROW1        = 1.00;
    public static double SPEED_PICKUP_ROW1    = 0.60;

    public static double SPEED_OPEN_GATE      = 0.80;
    public static double SPEED_RETURN_SHOOT1  = 1.00;

    public static double SPEED_TO_ROW2        = 1.00;
    public static double SPEED_PICKUP_ROW2    = 0.60;

    public static double SPEED_RETURN_SHOOT2  = 1.00;

    public static double SPEED_LEAVE          = 1.00;

    /* ================= ENUM STATE MACHINE ================= */
    private enum AutoState {
        MOVE_TO_SHOOT,
        SHOOT_PRELOAD,

        MOVE_ROW1,
        PICKUP_ROW1,
        OPEN_GATE_CURVE,
        RETURN_SHOOT1,
        SHOOT_CYCLE2,

        MOVE_ROW2,
        PICKUP_ROW2,
        RETURN_SHOOT2,
        SHOOT_CYCLE3,

        LEAVE,
        DONE
    }

    private AutoState state = AutoState.MOVE_TO_SHOOT;

    /* ================= POSES ================= */
    private final Pose startPos = new Pose(111, 135, Math.toRadians(0));
    private final Pose shootPos = new Pose(105.33, 108.4, Math.toRadians(42));

    private final Pose intakeRowOnePos = new Pose(95, 93, Math.toRadians(180));
    private final Pose intakePickUpRowOnePos = new Pose(125, 93, Math.toRadians(180));

    private final Pose openGate = new Pose(126.02, 90, Math.toRadians(180));
    private final Pose openGateControlPoint = new Pose(100, 78);

    private final Pose intakeRowTwoPos = new Pose(100, 70, Math.toRadians(180));
    private final Pose intakePickUpRowTwoPos = new Pose(131, 67, Math.toRadians(180));

    private final Pose leave = new Pose(96, 126.5, Math.toRadians(0));

    /* ================= PATHS ================= */
    private PathChain moveToShoot, moveToRow1, movePickUpRow1;
    private PathChain moveOpenGate, returnShoot1;

    private PathChain moveToRow2, movePickUpRow2, returnShoot2;
    private PathChain moveLeave;

    public void buildPaths() {

        moveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

        moveToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowOnePos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowOnePos.getHeading())
                .build();

        movePickUpRow1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowOnePos), new Point(intakePickUpRowOnePos)))
                .setLinearHeadingInterpolation(intakeRowOnePos.getHeading(), intakePickUpRowOnePos.getHeading())
                .build();

        moveOpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowOnePos),
                        new Point(openGateControlPoint.getX(), openGateControlPoint.getY()),
                        new Point(openGate)
                ))
                .setLinearHeadingInterpolation(intakePickUpRowOnePos.getHeading(), openGate.getHeading())
                .build();

        returnShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(openGate), new Point(shootPos)))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootPos.getHeading())
                .build();

        moveToRow2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowTwoPos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowTwoPos.getHeading())
                .build();

        movePickUpRow2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowTwoPos), new Point(intakePickUpRowTwoPos)))
                .setLinearHeadingInterpolation(intakeRowTwoPos.getHeading(), intakePickUpRowTwoPos.getHeading())
                .build();

        Point ctrl = new Point(100.929, 58.8972);

        returnShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowTwoPos),
                        ctrl,
                        new Point(shootPos)
                ))
                .setLinearHeadingInterpolation(intakePickUpRowTwoPos.getHeading(), shootPos.getHeading())
                .build();

        moveLeave = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(leave)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), leave.getHeading())
                .build();
    }

    /* ================= HARDWARE ================= */
    private DcMotorEx intake, shooter;
    private CRServo crLeft, crRight;
    private Servo spinServo, kickServo, adjustServo;
    private DistanceSensor distanceSensor;

    /* ================= CONSTANTS ================= */
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    public static double INTAKE_POWER = 0.8;
    public static double CR_INTAKE_POWER = 1.0;
    public static double DISTANCE_CM = 2.0;

    public static double INTAKE_0 = 0.145;
    public static double INTAKE_1 = 0.41;
    public static double INTAKE_2 = 0.7;

    private int ballCount = 0;
    private boolean lastDetected = false;
    private final ElapsedTime detectTimer = new ElapsedTime();

    public static double TICKS_PER_REV = 28.0;
    public static double RPM = 2350;
    public static double RPM_TOL = 75;
    public static double HOOD_POS = 0.75;

    public static double SHOOT_A = 0.56;
    public static double SHOOT_B = 0.28;
    public static double SHOOT_C = 0.00;

    public static double CR_L = 1.0;
    public static double CR_R = -1.0;

    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double SETTLE_SEC = 0.35;
    public static double KICK_MS = 500;
    public static double POST_MS = 500;

    enum Mode { IDLE, SHOOT }
    private Mode mode = Mode.IDLE;

    private enum ShootState {
        A, A_SETTLE, A_KICK, A_POST,
        B, B_SETTLE, B_KICK, B_POST,
        C, C_SETTLE, C_KICK, C_POST
    }

    private ShootState shootState = ShootState.A;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean prespin = false;

    /* ================= AUTO UPDATE ================= */
    public void autonomousPathUpdate() {

        updateMechanisms();

        switch (state) {

            /* ---- DRIVE TO SHOOT ---- */
            case MOVE_TO_SHOOT:
                follower.setMaxPower(SPEED_TO_SHOOT);
                prespin = true;
                setShooterRPM(RPM);

                follower.followPath(moveToShoot);
                state = AutoState.SHOOT_PRELOAD;
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(RPM, HOOD_POS);
                    state = AutoState.MOVE_ROW1;
                }
                break;

            /* ---- ROW 1 INTAKE ---- */
            case MOVE_ROW1:
                if (mode == Mode.IDLE) {
                    ballCount = 0;

                    follower.setMaxPower(SPEED_TO_ROW1);
                    runIntake(); // intake while moving
                    follower.followPath(moveToRow1);

                    state = AutoState.PICKUP_ROW1;
                }
                break;

            case PICKUP_ROW1:
                runIntake();

                if (!follower.isBusy() && ballCount >= 3) {
                    stopIntake();

                    follower.setMaxPower(SPEED_PICKUP_ROW1);
                    follower.followPath(movePickUpRow1);

                    state = AutoState.OPEN_GATE_CURVE;
                }
                break;

            /* ---- HIT LEVER ---- */
            case OPEN_GATE_CURVE:
                if (!follower.isBusy()) {

                    follower.setMaxPower(SPEED_OPEN_GATE);
                    follower.followPath(moveOpenGate);

                    state = AutoState.RETURN_SHOOT1;
                }
                break;

            /* ---- RETURN + SHOOT 2 ---- */
            case RETURN_SHOOT1:
                if (!follower.isBusy()) {

                    follower.setMaxPower(SPEED_RETURN_SHOOT1);
                    prespin = true;
                    setShooterRPM(RPM);

                    follower.followPath(returnShoot1);
                    state = AutoState.SHOOT_CYCLE2;
                }
                break;

            case SHOOT_CYCLE2:
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(RPM, HOOD_POS);

                    state = AutoState.MOVE_ROW2;
                }
                break;

            /* ---- ROW 2 INTAKE ---- */
            case MOVE_ROW2:
                if (mode == Mode.IDLE) {
                    ballCount = 0;

                    follower.setMaxPower(SPEED_TO_ROW2);
                    runIntake();
                    follower.followPath(moveToRow2);

                    state = AutoState.PICKUP_ROW2;
                }
                break;

            case PICKUP_ROW2:
                runIntake();

                if (!follower.isBusy() && ballCount >= 3) {
                    stopIntake();

                    follower.setMaxPower(SPEED_PICKUP_ROW2);
                    follower.followPath(movePickUpRow2);

                    state = AutoState.RETURN_SHOOT2;
                }
                break;

            /* ---- RETURN + SHOOT 3 ---- */
            case RETURN_SHOOT2:
                if (!follower.isBusy()) {

                    follower.setMaxPower(SPEED_RETURN_SHOOT2);
                    prespin = true;
                    setShooterRPM(RPM);

                    follower.followPath(returnShoot2);
                    state = AutoState.SHOOT_CYCLE3;
                }
                break;

            case SHOOT_CYCLE3:
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(RPM, HOOD_POS);

                    state = AutoState.LEAVE;
                }
                break;

            /* ---- LEAVE ---- */
            case LEAVE:
                if (mode == Mode.IDLE) {

                    follower.setMaxPower(SPEED_LEAVE);
                    follower.followPath(moveLeave);

                    state = AutoState.DONE;
                }
                break;

            case DONE:
                if (!follower.isBusy()) stopAllMechanisms();
                break;
        }
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

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo = hardwareMap.get(Servo.class, "spin");
        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        adjustServo = hardwareMap.get(Servo.class, "adjust_servo");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        kickServo.setPosition(KICK_IDLE);
        spinServo.setPosition(INTAKE_0);
        adjustServo.setPosition(HOOD_POS);
    }

    @Override
    public void start() {
        state = AutoState.MOVE_TO_SHOOT;
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", state);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("RPM", getShooterRPM());
        telemetry.update();
    }

    /* ================= INTAKE ================= */
    private void runIntake() {

        if (ballCount >= 3) return;

        intake.setPower(INTAKE_POWER);
        setCR(CR_INTAKE_POWER, CR_INTAKE_POWER);

        boolean detected = distanceSensor.getDistance(DistanceUnit.CM) <= DISTANCE_CM;
        if (detected && !lastDetected && detectTimer.seconds() > 0.4) {
            ballCount++;
            detectTimer.reset();
        }
        lastDetected = detected;

        spinServo.setPosition(
                ballCount == 0 ? INTAKE_0 :
                        ballCount == 1 ? INTAKE_1 : INTAKE_2
        );
    }

    private void stopIntake() {
        intake.setPower(0);
        setCR(0, 0);
    }

    /* ================= SHOOT FSM ================= */
    private void startShoot(double rpm, double hood) {
        mode = Mode.SHOOT;
        shootState = ShootState.A;
        shootTimer.reset();
        adjustServo.setPosition(hood);
        setShooterRPM(rpm);
    }

    private void updateMechanisms() {
        if (mode == Mode.SHOOT) runShoot();
        else hold();
    }

    private void runShoot() {
        switch (shootState) {

            case A:
                spinServo.setPosition(SHOOT_A);
                setCR(CR_L, CR_R);
                shootTimer.reset();
                shootState = ShootState.A_SETTLE;
                break;

            case A_SETTLE:
                if (shootTimer.seconds() >= SETTLE_SEC && shooterAtSpeed()) {
                    kickServo.setPosition(KICK_ACTIVE);
                    shootTimer.reset();
                    shootState = ShootState.A_KICK;
                }
                break;

            case A_KICK:
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    shootTimer.reset();
                    shootState = ShootState.A_POST;
                }
                break;

            case A_POST:
                if (shootTimer.milliseconds() >= POST_MS)
                    shootState = ShootState.B;
                break;

            case B:
                spinServo.setPosition(SHOOT_B);
                shootTimer.reset();
                shootState = ShootState.B_SETTLE;
                break;

            case B_SETTLE:
                if (shootTimer.seconds() >= SETTLE_SEC && shooterAtSpeed()) {
                    kickServo.setPosition(KICK_ACTIVE);
                    shootTimer.reset();
                    shootState = ShootState.B_KICK;
                }
                break;

            case B_KICK:
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    shootTimer.reset();
                    shootState = ShootState.B_POST;
                }
                break;

            case B_POST:
                if (shootTimer.milliseconds() >= POST_MS)
                    shootState = ShootState.C;
                break;

            case C:
                spinServo.setPosition(SHOOT_C);
                shootTimer.reset();
                shootState = ShootState.C_SETTLE;
                break;

            case C_SETTLE:
                if (shootTimer.seconds() >= SETTLE_SEC && shooterAtSpeed()) {
                    kickServo.setPosition(KICK_ACTIVE);
                    shootTimer.reset();
                    shootState = ShootState.C_KICK;
                }
                break;

            case C_KICK:
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    shootTimer.reset();
                    shootState = ShootState.C_POST;
                }
                break;

            case C_POST:
                if (shootTimer.milliseconds() >= POST_MS) {
                    setCR(0, 0);
                    setShooterRPM(0);
                    spinServo.setPosition(INTAKE_0);
                    mode = Mode.IDLE;
                }
                break;
        }
    }

    private void hold() {
        stopIntake();
        if (!prespin) setShooterRPM(0);
        kickServo.setPosition(KICK_IDLE);
    }

    private void stopAllMechanisms() {
        stopIntake();
        setShooterRPM(0);
        spinServo.setPosition(INTAKE_0);
        kickServo.setPosition(KICK_IDLE);
        mode = Mode.IDLE;
        prespin = false;
    }

    /* ================= HELPERS ================= */
    private void setCR(double l, double r) {
        crLeft.setPower(l);
        crRight.setPower(r);
    }

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
