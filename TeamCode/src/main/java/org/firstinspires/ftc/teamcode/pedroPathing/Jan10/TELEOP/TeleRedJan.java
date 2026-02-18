package org.firstinspires.ftc.teamcode.pedroPathing.Jan10.TELEOP;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "TELE RED Leave", group = "BackUp")
@Config
public class TeleRedJan extends OpMode {

    /* ================= DRIVE ================= */
    private Follower follower;
    private final Pose startPose = new Pose(95, 9, Math.toRadians(90));

    public static Pose TARGET_A = new Pose(107.3, 106, Math.toRadians(42));
    public static Pose TARGET_B = new Pose(105.331136738056, 108.4, Math.toRadians(42));
    public static Pose TARGET_C = new Pose(93.5, 14.38, Math.toRadians(71));

    public static double RPM_A = 2500;
    public static double RPM_B = 2200;
    public static double RPM_C = 3500;

    public static double HOOD_A = 0.75;
    public static double HOOD_B = 1.0;
    public static double HOOD_C = 0.5;

    private Pose activeTarget = null;
    private boolean navigating = false;

    private double activeRPM = 0;
    private double activeHood = HOOD_A;

    public static double POS_TOL = 2.0;
    public static double HEAD_TOL_DEG = 0.01;

    private boolean lastXDrive = false;
    private boolean lastYDrive = false;
    private boolean lastBDrive = false;

    /* ================= SLOW MODE ================= */
    private boolean slowMode = false;
    private boolean lastSlowToggle = false;

    public static double SLOW_MULT = 0.35;
    public static double TRIGGER_MIN_MULT = 0.25;

    /* ================= HARDWARE ================= */
    private DcMotorEx intake, shooter;
    private CRServo crLeft, crRight;
    private Servo spinServo, kickServo, adjustServo;
    private DistanceSensor distanceSensor;

    /* ================= INTAKE ================= */
    public static double INTAKE_POWER = 0.8;
    public static double CR_INTAKE_POWER = 1.0;
    public static double DISTANCE_CM = 3.0;

    /* ================= SHOOTER ================= */
    public static double TICKS_PER_REV = 28.0;
    public static double SHOOTER_P = 70.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 8.0;
    public static double SHOOTER_F = 13.5;
    public static double RPM_TOLERANCE = 75;

    /* ================= SPIN POSITIONS ================= */
    public static double INTAKE_0 = 0.145;
    public static double INTAKE_1 = 0.41;
    public static double INTAKE_2 = 0.7;

    /* ================= PWM ================= */
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    public static double SHOOT_A = 0.56;
    public static double SHOOT_B = 0.28;
    public static double SHOOT_C = 0.00;

    /* ================= CR SHOOT ================= */
    public static double CR_A_L = 1.0, CR_A_R = -1.0;
    public static double CR_B_L = 1.0, CR_B_R = -1.0;
    public static double CR_C_L = 1.0, CR_C_R = -1.0;

    /* ================= KICKER ================= */
    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double SETTLE_SEC = 0.35;
    public static double KICK_MS = 1000;
    public static double POST_MS = 500;

    /* ================= HOOD ================= */
    public static final double HOOD_MIN = 0.73;
    public static final double HOOD_MAX = 1.0;

    /* ================= MODES ================= */
    private enum Mode { IDLE, INTAKE, SHOOT }
    private Mode mode = Mode.IDLE;

    private int ballCount = 0;
    private boolean lastDetected = false;
    private final ElapsedTime detectTimer = new ElapsedTime();

    private enum ShootState {
        A, A_SETTLE, A_KICK, A_POST,
        B, B_SETTLE, B_KICK, B_POST,
        C, C_SETTLE, C_KICK, C_POST
    }

    private ShootState shootState = ShootState.A;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean lastA = false, lastXShoot = false;

    /* ================= INIT ================= */
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F)
        );

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo = hardwareMap.get(Servo.class, "spin");
        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        adjustServo = hardwareMap.get(Servo.class, "adjust_servo");

        if (spinServo instanceof PwmControl) {
            ((PwmControl) spinServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        }
        if (adjustServo instanceof PwmControl) {
            ((PwmControl) adjustServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        }

        kickServo.setPosition(KICK_IDLE);
        spinServo.setPosition(INTAKE_0);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        follower.startTeleopDrive();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {

        /* ===== SLOW MODE TOGGLE ===== */
        boolean slowTogglePressed = gamepad1.a && !lastSlowToggle;
        if (slowTogglePressed) slowMode = !slowMode;
        lastSlowToggle = gamepad1.a;

        /* ===== TARGET SELECT ===== */
        boolean xPressed = gamepad1.x && !lastXDrive;
        boolean yPressed = gamepad1.y && !lastYDrive;
        boolean bPressed = gamepad1.b && !lastBDrive;

        if (!navigating) {
            if (xPressed) {
                activeTarget = TARGET_A;
                activeRPM = RPM_A;
                activeHood = HOOD_A;
                navigating = true;
            } else if (yPressed) {
                activeTarget = TARGET_B;
                activeRPM = RPM_B;
                activeHood = HOOD_B;
                navigating = true;
            } else if (bPressed) {
                activeTarget = TARGET_C;
                activeRPM = RPM_C;
                activeHood = HOOD_C;
                navigating = true;
            }

            if (navigating) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(activeTarget)
                                ))
                                .setLinearHeadingInterpolation(
                                        follower.getPose().getHeading(),
                                        activeTarget.getHeading()
                                )
                                .build()
                );
            }
        }

        lastXDrive = gamepad1.x;
        lastYDrive = gamepad1.y;
        lastBDrive = gamepad1.b;

        if (navigating || mode == Mode.SHOOT) {
            setShooterRPM(activeRPM);
        }

        if ((gamepad1.left_bumper || gamepad1.right_bumper) && navigating) {
            navigating = false;
            activeTarget = null;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        if (navigating) {
            follower.update();
            if (arrived()) {
                navigating = false;
                activeTarget = null;
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
        } else {
            double trigger = gamepad1.right_trigger;
            double triggerMult = 1.0 - trigger * (1.0 - TRIGGER_MIN_MULT);

            double finalMult = slowMode
                    ? Math.min(SLOW_MULT, triggerMult)
                    : triggerMult;

            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y * finalMult,
                    -gamepad1.left_stick_x * finalMult,
                    -gamepad1.right_stick_x * finalMult,
                    false
            );
            follower.update();
        }

        boolean aPressed = gamepad2.dpad_down && !lastA;
        boolean xShootPressed = gamepad2.dpad_up && !lastXShoot;
        lastA = gamepad2.dpad_down;
        lastXShoot = gamepad2.dpad_up;

        if (aPressed) mode = Mode.INTAKE;

        if (xShootPressed && ballCount > 0) {
            mode = Mode.SHOOT;
            shootState = ShootState.A;
            shootTimer.reset();
        }

        adjustServo.setPosition(
                Math.max(HOOD_MIN, Math.min(HOOD_MAX, activeHood))
        );

        switch (mode) {
            case INTAKE: runIntake(); break;
            case SHOOT: runShoot(); break;
            default: hold(); break;
        }

        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Mode", mode);
        telemetry.update();
    }

    /* ================= INTAKE ================= */
    private void runIntake() {
        intake.setPower(INTAKE_POWER);
        setCR(CR_INTAKE_POWER, CR_INTAKE_POWER);

        boolean detected = distanceSensor.getDistance(DistanceUnit.CM) <= DISTANCE_CM;
        if (detected && !lastDetected && detectTimer.seconds() > 0.4 && ballCount < 3) {
            ballCount++;
            detectTimer.reset();
        }
        lastDetected = detected;

        spinServo.setPosition(
                ballCount == 0 ? INTAKE_0 :
                        ballCount == 1 ? INTAKE_1 : INTAKE_2
        );
    }

    /* ================= SHOOT ================= */
    private void runShoot() {
        intake.setPower(0);

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
                if (shootTimer.milliseconds() >= POST_MS) shootState = ShootState.C;
                break;

            case C:
                spinServo.setPosition(SHOOT_C);
                setCR(CR_C_L, CR_C_R);
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
                    ballCount = 0;
                    mode = Mode.IDLE;
                }
                break;
        }
    }

    /* ================= HELPERS ================= */
    private void hold() {
        intake.setPower(0);
        setCR(0, 0);

        if (mode == Mode.IDLE && !navigating) {
            setShooterRPM(0);
        }

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

    private double getShooterRPM() {
        return shooter.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    private boolean shooterAtSpeed() {
        return Math.abs(getShooterRPM() - activeRPM) < RPM_TOLERANCE;
    }

    private boolean arrived() {
        if (activeTarget == null) return false;

        Pose p = follower.getPose();
        double dx = activeTarget.getX() - p.getX();
        double dy = activeTarget.getY() - p.getY();
        double dist = Math.hypot(dx, dy);

        double headingErr = Math.toDegrees(
                Math.atan2(
                        Math.sin(p.getHeading() - activeTarget.getHeading()),
                        Math.cos(p.getHeading() - activeTarget.getHeading())
                )
        );

        return dist < POS_TOL && Math.abs(headingErr) < HEAD_TOL_DEG;
    }
}