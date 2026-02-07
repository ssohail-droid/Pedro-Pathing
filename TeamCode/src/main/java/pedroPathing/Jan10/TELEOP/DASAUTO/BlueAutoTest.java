package pedroPathing.Jan10.TELEOP.DASAUTO;

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

@Autonomous(name = "Blue Auto (Legacy)", group = "Main")
public class BlueAutoTest extends OpMode {

    /* ================= DRIVE ================= */
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPos = new Pose(32, 135, Math.toRadians(180));
    private final Pose shootPos = new Pose(41, 111.05, Math.toRadians(140));

    private final Pose intakeRowOnePos = new Pose(54, 78, Math.toRadians(0));
    /// ///////////////////////////////////////////////^^from ball
    private final Pose intakePickUpRowOnePos = new Pose(22, 78, Math.toRadians(0));
    /// /////////////////////////////////////////////////////^^from gg

    private final Pose intakeRowTwoPos = new Pose(54, 55, Math.toRadians(0));
    /// ///////////////////////////////////////////////^^from ball
    private final Pose intakePickUpRowTwoPos = new Pose(24.5 , 55, Math.toRadians(0));
    /// /////////////////////////////////////////////////////^^from gg

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

    /* ================= CR SHOOT ================= */
    public static double CR_L = 1.0;
    public static double CR_R = -1.0;

    /* ================= KICKER ================= */
    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double SETTLE_SEC = 0.35;
    public static double KICK_MS = 500;
    public static double POST_MS = 500;

    /* ================= STATE ================= */
    private enum Mode { IDLE, SHOOT }
    private Mode mode = Mode.IDLE;

    private enum ShootState {
        A, A_SETTLE, A_KICK, A_POST,
        B, B_SETTLE, B_KICK, B_POST,
        C, C_SETTLE, C_KICK, C_POST
    }

    private ShootState shootState = ShootState.A;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private boolean prespin = false;

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

            case 0: // Drive to shoot
                prespin = true;
                setShooterRPM(RPM);
                adjustServo.setPosition(HOOD_POS);
                follower.followPath(moveOne);
                setPathState(1);
                break;

            case 1: // Shoot preload
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(RPM, HOOD_POS);
                    setPathState(2);
                }
                break;

            case 2: // After preload shot → Row 1
                updateMechanisms();
                if (mode == Mode.IDLE) {
                    ballCount = 0;
                    follower.followPath(moveTwo);
                    setPathState(3);
                }
                break;

            case 3: // Start Row 1 intake
                if (!follower.isBusy()) {
                    prespin = true;
                    setShooterRPM(RPM);
                    follower.setMaxPower(0.2);
                    follower.followPath(moveThree);
                    setPathState(4);
                }
                break;

            case 4: // Intake Row 1
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

            case 5: // Shoot Row 1
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(RPM, HOOD_POS);
                    setPathState(6);
                }
                break;

            case 6: // After Row 1 shot → Row 2
                updateMechanisms();
                if (mode == Mode.IDLE) {
                    ballCount = 0;
                    follower.followPath(moveFive);
                    setPathState(7);
                }
                break;

            case 7: // Start Row 2 intake
                if (!follower.isBusy()) {
                    prespin = true;
                    setShooterRPM(RPM);
                    follower.setMaxPower(0.2);
                    follower.followPath(moveSix);
                    setPathState(8);
                }
                break;

            case 8: // Intake Row 2
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

            case 9: // Shoot Row 2
                if (!follower.isBusy()) {
                    prespin = false;
                    startShoot(RPM, HOOD_POS);
                    setPathState(10);
                }
                break;

            case 10: // End
                updateMechanisms();
                if (mode == Mode.IDLE) {
                    setPathState(-1);
                }
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

        telemetry.addData("Path State", pathState);
        telemetry.addData("Mode", mode);
        telemetry.addData("RPM", getShooterRPM());
        telemetry.addData("Ball Count", ballCount);
        telemetry.update();
    }

    /* ================= MECHANISMS ================= */

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