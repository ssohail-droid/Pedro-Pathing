package org.firstinspires.ftc.teamcode.pedroPathing.Jan10.TELEOP.DASAUTO;

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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

// Final
//Lock This Code NO CHANGE EVER
@Autonomous(name = "Blue Auto Joint", group = "Joint")
public class BlueAutoJointPathingV3 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    /* ================= AUTO STATE MACHINE ================= */

    private enum AutoState {
        MOVE_TO_SHOOT,
        SHOOT_START,

        MOVE_ROW1,
        PICKUP_ROW1,
        OPEN_GATE_CURVE,
        RETURN_SHOOT1,
        SHOOT_AFTER_ROW1,

        MOVE_ROW2,
        PICKUP_ROW2,
        DRIVE_PICKUP_ROW2,
        RETURN_SHOOT2,
        SHOOT_AFTER_ROW2,

        LEAVE,
        DONE
    }

    private AutoState state = AutoState.MOVE_TO_SHOOT;

    /* ================= POSES ================= */
    private final Pose startPos = new Pose(32, 135, Math.toRadians(180));
    private final Pose shootPos = new Pose(41, 111.05, Math.toRadians(140));

    private final Pose intakeRowOnePos = new Pose(52, 78, Math.toRadians(0));
    private final Pose intakePickUpRowOnePos = new Pose(16, 78, Math.toRadians(0));

    private final Pose openGate = new Pose(17.5, 71, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(35, 79, Math.toRadians(0));

    private final Pose intakeRowTwoPos = new Pose(54, 52, Math.toRadians(0));
    private final Pose intakePickUpRowTwoPos = new Pose(7, 52, Math.toRadians(0));

    private final Pose leave = new Pose(50, 123.76, Math.toRadians(180));

    /* ================= PATHS ================= */

    private PathChain moveToShoot, moveToIntakeRowOne, moveToPickUpRowOne;
    private PathChain moveToOpenGate, moveRoundTwoShoot;

    private PathChain moveIntakeRowTwo, movePickUpRowTwo, moveShootRowTwo;
    private PathChain moveLeave;

    /* ================= HARDWARE ================= */

    private DcMotorEx intake, shooter;
    private CRServo crLeft, crRight;
    private Servo spinServo, kickServo, adjustServo;
    private DistanceSensor distanceSensor;

    /* ================= PWM ================= */

    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    /* ================= INTAKE ================= */

    public static double INTAKE_POWER = 0.9;
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
    public static double RPM = 2350;
    public static double RPM_TOL = 75;
    public static double HOOD_POS = 0.75;

    public static double SHOOT_A = 0.56;
    public static double SHOOT_B = 0.28;
    public static double SHOOT_C = 0.00;

    /* ================= KICKER ================= */

    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double SETTLE_SEC = 0.35;
    public static double KICK_MS = 500;
    public static double POST_MS = 500; // <<< USED NOW

    /* ================= SHOOT FSM ================= */

    private enum Mode { IDLE, SHOOT }
    private Mode mode = Mode.IDLE;

    private enum ShootState {
        A, A_SETTLE, A_KICK, A_POST,
        B, B_SETTLE, B_KICK, B_POST,
        C, C_SETTLE, C_KICK, C_POST
    }

    private ShootState shootState = ShootState.A;
    private final ElapsedTime shootTimer = new ElapsedTime();

    /* ================= BUILD PATHS ================= */

    public void buildPaths() {

        moveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

        moveToIntakeRowOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowOnePos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowOnePos.getHeading())
                .build();

        moveToPickUpRowOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowOnePos), new Point(intakePickUpRowOnePos)))
                .setLinearHeadingInterpolation(intakeRowOnePos.getHeading(), intakePickUpRowOnePos.getHeading())
                .build();

        moveToOpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowOnePos),
                        new Point(openGateControlPoint.getX(), openGateControlPoint.getY()),
                        new Point(openGate)
                ))
                .setLinearHeadingInterpolation(intakePickUpRowOnePos.getHeading(), openGate.getHeading())
                .build();

        moveRoundTwoShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(openGate), new Point(shootPos)))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootPos.getHeading())
                .build();

        moveIntakeRowTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowTwoPos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowTwoPos.getHeading())
                .build();

        movePickUpRowTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowTwoPos), new Point(intakePickUpRowTwoPos)))
                .setLinearHeadingInterpolation(intakeRowTwoPos.getHeading(), intakePickUpRowTwoPos.getHeading())
                .build();

        Point control = new Point(60, 58.8972);

        moveShootRowTwo = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakePickUpRowTwoPos),
                        control,
                        new Point(shootPos)
                ))
                .setLinearHeadingInterpolation(intakePickUpRowTwoPos.getHeading(), shootPos.getHeading())
                .build();

        moveLeave = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(leave)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), leave.getHeading())
                .build();
    }

    /* ================= AUTO UPDATE ================= */

    public void autonomousPathUpdate() {

        switch (state) {

            case MOVE_TO_SHOOT:
                follower.followPath(moveToShoot);
                state = AutoState.SHOOT_START;
                break;

            case SHOOT_START:
                if (!follower.isBusy()) {
                    if (mode == Mode.IDLE) startShoot();
// MOVE_ROW1 will only start once mode returns to IDLE
                    state = AutoState.MOVE_ROW1;
                }
                break;

            case MOVE_ROW1:
                if (mode == Mode.IDLE) {
                    follower.followPath(moveToIntakeRowOne);
                    state = AutoState.PICKUP_ROW1;
                }
                break;

            case PICKUP_ROW1:
                runIntake();
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(0.6);
                    follower.followPath(moveToPickUpRowOne);
                    state = AutoState.OPEN_GATE_CURVE;
                }
                break;

            case OPEN_GATE_CURVE:
                runIntake();
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(moveToOpenGate);
                    state = AutoState.RETURN_SHOOT1;
                }
                break;

            case RETURN_SHOOT1:
                if (!follower.isBusy()) {
                    follower.followPath(moveRoundTwoShoot);
                    state = AutoState.SHOOT_AFTER_ROW1;
                }
                break;

            case SHOOT_AFTER_ROW1:
                if (!follower.isBusy()) {
                    if (mode == Mode.IDLE) startShoot();
                    state = AutoState.MOVE_ROW2;
                }
                break;

            case MOVE_ROW2:

                if (mode == Mode.IDLE) {
                    follower.followPath(moveIntakeRowTwo);

                    detectTimer.reset();
                    lastDetected = false;

                    state = AutoState.PICKUP_ROW2;
                }

                break;


            case PICKUP_ROW2:

                if (!follower.isBusy()) {

                    follower.setMaxPower(0.6); // optional like row 1
                    follower.followPath(movePickUpRowTwo);



                    state = AutoState.DRIVE_PICKUP_ROW2;
                }

                break;


            case DRIVE_PICKUP_ROW2:

                runIntake();

                if (ballCount >= 3) {
                    stopIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(moveShootRowTwo);
                    state = AutoState.RETURN_SHOOT2;
                    break;
                }

                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(moveShootRowTwo);
                    state = AutoState.RETURN_SHOOT2;
                }

                break;


            case RETURN_SHOOT2:

// Prespin shooter while driving back (makes shooting instant)
                setShooterRPM(RPM);

                if (!follower.isBusy()) {
                    state = AutoState.SHOOT_AFTER_ROW2;
                }
                break;


            case SHOOT_AFTER_ROW2:

// Shoot only once we fully arrived
                if (mode == Mode.IDLE) {
                    startShoot();
                    state = AutoState.LEAVE;
                }
                break;

            case LEAVE:
                if (mode == Mode.IDLE) {
                    follower.followPath(moveLeave);
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                stopIntake();
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
        state = AutoState.MOVE_TO_SHOOT;
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        updateMechanisms(); // <<< REQUIRED

        telemetry.addData("State", state);
        telemetry.addData("Mode", mode);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("RPM", getShooterRPM());
        telemetry.update();
    }

    /* ================= MECHANISMS ================= */

    private void updateMechanisms() {
        if (mode == Mode.SHOOT) runShoot();
    }

    private void runIntake() {

        intake.setPower(INTAKE_POWER);
        crLeft.setPower(CR_INTAKE_POWER);
        crRight.setPower(CR_INTAKE_POWER);

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
        crLeft.setPower(0);
        crRight.setPower(0);
    }

    /* ================= SHOOT ================= */

    private void startShoot() {
        mode = Mode.SHOOT;
        shootState = ShootState.A;
        shootTimer.reset();
        setShooterRPM(RPM);
    }

    private void runShoot() {

        switch (shootState) {

            case A:
                spinServo.setPosition(SHOOT_A);
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
                    shootState = ShootState.A_POST; // <<< POST
                }
                break;

            case A_POST:
                if (shootTimer.milliseconds() >= POST_MS) {
                    shootState = ShootState.B;
                }
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
                    shootState = ShootState.B_POST; // <<< POST
                }
                break;

            case B_POST:
                if (shootTimer.milliseconds() >= POST_MS) {
                    shootState = ShootState.C;
                }
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
                    shootState = ShootState.C_POST; // <<< POST
                }
                break;

            case C_POST:
                if (shootTimer.milliseconds() >= POST_MS) {
                    setShooterRPM(0);
                    mode = Mode.IDLE;
                    ballCount = 0;
                    spinServo.setPosition(INTAKE_0);
                }
                break;
        }
    }

    /* ================= HELPERS ================= */

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