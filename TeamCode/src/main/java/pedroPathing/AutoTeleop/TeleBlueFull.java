package pedroPathing.AutoTeleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "TeleBlue Drive + Intake + Shoot + Hood", group = "Main")
@Config
public class TeleBlueFull extends OpMode {

    // ================= DRIVE =================
    private Follower follower;
    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    public static Pose targetPose = new Pose(24, 24, Math.toRadians(50));
    private boolean navigating = false;
    private boolean lastXDrive = false;

    public static double POS_TOL = 2.0;
    public static double HEAD_TOL_DEG = 5.0;

    // ================= HARDWARE =================
    private DcMotorEx intake, shooter;
    private CRServo crLeft, crRight;
    private Servo spinServo, kickServo, adjustServo;
    private DistanceSensor distanceSensor;

    // ================= DASHBOARD =================
    // Intake
    public static double INTAKE_POWER = 0.4;
    public static double CR_INTAKE_POWER = 1.0;
    public static double DISTANCE_CM = 2.0;

    // Shooter
    public static double SHOOTER_RPM = 1500;
    public static double TICKS_PER_REV = 28.0;

    // Spin positions (intake)
    public static double INTAKE_0 = 0.18;
    public static double INTAKE_1 = 0.46;
    public static double INTAKE_2 = 0.63;

    // Spin positions (shoot)
    public static double SHOOT_A = 0.56;
    public static double SHOOT_B = 0.32;
    public static double SHOOT_C = 0.04;

    // CR shoot profiles
    public static double CR_A_L = 1.0, CR_A_R = 1.0;
    public static double CR_B_L = 1.0, CR_B_R = 1.0;
    public static double CR_C_L = 1.0, CR_C_R = 1.0;

    // Kicker
    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double SETTLE_SEC = 2.0;
    public static double KICK_MS = 1000;
    public static double POST_MS = 500;

    // Hood (dashboard)
    public static double HOOD_TARGET = 0.50;
    public static final double HOOD_MIN = 0.40;
    public static final double HOOD_MAX = 0.60;

    // PWM
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    // ================= MODES =================
    private enum Mode { IDLE, INTAKE, SHOOT }
    private Mode mode = Mode.IDLE;

    // Intake state
    private int ballCount = 0;
    private boolean lastDetected = false;
    private final ElapsedTime detectTimer = new ElapsedTime();

    // Shoot state
    private enum ShootState { A, A_SETTLE, A_KICK, A_POST,
        B, B_SETTLE, B_KICK, B_POST,
        C, C_SETTLE, C_KICK, C_POST }
    private ShootState shootState = ShootState.A;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean lastA = false, lastXShoot = false;

    // ================= INIT =================
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

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

        kickServo.setPosition(KICK_IDLE);
        spinServo.setPosition(INTAKE_0);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        follower.startTeleopDrive();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        // ---------- DRIVE GO-TO ----------
        if (gamepad1.x && !lastXDrive && !navigating) {
            navigating = true;
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
        lastXDrive = gamepad1.x;

        if ((gamepad1.left_bumper || gamepad1.right_bumper) && navigating) {
            navigating = false;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        if (navigating) {
            follower.update();
            if (arrived()) {
                navigating = false;
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
        } else {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // ---------- MODE BUTTONS ----------
        boolean aPressed = gamepad1.a && !lastA;
        boolean xShootPressed = gamepad1.x && !lastXShoot;
        lastA = gamepad1.a;
        lastXShoot = gamepad1.x;

        if (aPressed) mode = Mode.INTAKE;
        if (xShootPressed && ballCount > 0) {
            mode = Mode.SHOOT;
            shootState = ShootState.A;
            shootTimer.reset();
        }

        // ---------- HOOD (ALWAYS) ----------
        double hoodSafe = Math.max(HOOD_MIN, Math.min(HOOD_MAX, HOOD_TARGET));
        adjustServo.setPosition(hoodSafe);

        // ---------- MODES ----------
        switch (mode) {
            case INTAKE: runIntake(); break;
            case SHOOT:  runShoot();  break;
            default: hold(); break;
        }

        telemetry.addData("Mode", mode);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Hood", hoodSafe);
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }

    // ================= INTAKE =================
    private void runIntake() {
        intake.setPower(INTAKE_POWER);
        setCR(CR_INTAKE_POWER, CR_INTAKE_POWER);
        setShooterRPM(0);

        double d = distanceSensor.getDistance(DistanceUnit.CM);
        boolean detected = d <= DISTANCE_CM;

        if (detected && !lastDetected && detectTimer.seconds() > 0.4 && ballCount < 2) {
            ballCount++;
            detectTimer.reset();
        }
        lastDetected = detected;

        spinServo.setPosition(ballCount == 0 ? INTAKE_0 :
                ballCount == 1 ? INTAKE_1 : INTAKE_2);
    }

    // ================= SHOOT =================
    private void runShoot() {
        intake.setPower(0);
        setShooterRPM(SHOOTER_RPM);

        switch (shootState) {

            case A:
                spinServo.setPosition(SHOOT_A);
                setCR(CR_A_L, CR_A_R);
                shootTimer.reset();
                shootState = ShootState.A_SETTLE;
                break;

            case A_SETTLE:
                if (shootTimer.seconds() >= SETTLE_SEC) {
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
                if (shootTimer.milliseconds() >= POST_MS) shootState = ShootState.B;
                break;

            case B:
                spinServo.setPosition(SHOOT_B);
                setCR(CR_B_L, CR_B_R);
                shootTimer.reset();
                shootState = ShootState.B_SETTLE;
                break;

            case B_SETTLE:
                if (shootTimer.seconds() >= SETTLE_SEC) {
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
                if (shootTimer.milliseconds() >= POST_MS) shootState = ShootState.C;
                break;

            case C:
                spinServo.setPosition(SHOOT_C);
                setCR(CR_C_L, CR_C_R);
                shootTimer.reset();
                shootState = ShootState.C_SETTLE;
                break;

            case C_SETTLE:
                if (shootTimer.seconds() >= SETTLE_SEC) {
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
                    spinServo.setPosition(INTAKE_0);
                    setCR(0, 0);
                    setShooterRPM(0);
                    ballCount = 0;
                    mode = Mode.IDLE;
                }
                break;
        }
    }

    private void hold() {
        intake.setPower(0);
        setCR(0, 0);
        setShooterRPM(0);
        spinServo.setPosition(INTAKE_0);
        kickServo.setPosition(KICK_IDLE);
    }

    private void setCR(double l, double r) {
        crLeft.setPower(l);
        crRight.setPower(r);
    }

    private void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * TICKS_PER_REV) / 60.0);
    }

    private boolean arrived() {
        Pose p = follower.getPose();
        double dx = targetPose.getX() - p.getX();
        double dy = targetPose.getY() - p.getY();
        double dist = Math.hypot(dx, dy);
        double headingErr = Math.toDegrees(p.getHeading() - targetPose.getHeading());
        return dist < POS_TOL && Math.abs(headingErr) < HEAD_TOL_DEG;
    }
}
