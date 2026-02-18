package org.firstinspires.ftc.teamcode.pedroPathing.Jan10.TELEOP.TESTOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

//@TeleOp(name = "Intake + Shoot (Button Modes)", group = "Main")
@Config
public class IntakeShootSystem extends OpMode {

    // ===== HARDWARE =====
    private DcMotorEx intake, shooter;
    private CRServo crLeft, crRight;
    private Servo spinServo, kickServo;
    private DistanceSensor distanceSensor;

    // ===== DASHBOARD =====
    // Intake mode
    public static double INTAKE_POWER = 0.4;
    public static double INTAKE_CR_POWER = 1.0;
    public static double DISTANCE_CM = 2.0;
    public static double DETECT_COOLDOWN_SEC = 0.5;

    // Shooter
    public static double SHOOTER_RPM = 3000;
    public static double TICKS_PER_REV = 28.0;

    // Intake spin staging
    public static double INTAKE_0 = 0.18;
    public static double INTAKE_1 = 0.46;
    public static double INTAKE_2 = 0.63;

    // Shoot spin sequence
    public static double SHOOT_A = 0.56;
    public static double SHOOT_B = 0.32;
    public static double SHOOT_C = 0.04;

    // Shoot CR profiles (✅ independent per stage)
    public static double SHOOT_CR_A_L = 1.0, SHOOT_CR_A_R = -1.0;
    public static double SHOOT_CR_B_L = 1.0, SHOOT_CR_B_R =-1.0;
    public static double SHOOT_CR_C_L = 1.0, SHOOT_CR_C_R = -1.0;

    // Timing
    public static double SETTLE_SEC = 2.0;
    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double KICK_MS = 1000;
    public static double POST_MS = 500;

    // 5-turn servo PWM range
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    // ===== MODES =====
    private enum Mode { IDLE, INTAKE, SHOOT }
    private Mode mode = Mode.IDLE;

    // ===== INTAKE STATE =====
    private int ballCount = 0;                  // 0..2
    private boolean lastDetected = false;
    private final ElapsedTime detectCooldown = new ElapsedTime();

    // ===== SHOOT STATE =====
    private enum ShootState {
        A, A_SETTLE, A_KICK, A_POST,
        B, B_SETTLE, B_KICK, B_POST,
        C, C_SETTLE, C_KICK, C_POST
    }
    private ShootState shootState = ShootState.A;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // ===== EDGE DETECT =====
    private boolean lastA = false, lastX = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        // ✅ Reverse right so +power pulls the same direction as left
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo = hardwareMap.get(Servo.class, "spin");
        if (spinServo instanceof PwmControl) {
            ((PwmControl) spinServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        }

        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        kickServo.setPosition(KICK_IDLE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        // Safe defaults
        spinServo.setPosition(INTAKE_0);
        intake.setPower(0);
        setShooterRPM(0);
        setCR(0, 0);

        detectCooldown.reset();
        shootTimer.reset();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a && !lastA;
        boolean xPressed = gamepad1.x && !lastX;
        lastA = gamepad1.a;
        lastX = gamepad1.x;

        // A = go to INTAKE mode anytime
        if (aPressed) {
            mode = Mode.INTAKE;
        }

        // X = start SHOOT macro only if we actually have something
        if (xPressed && ballCount > 0) {
            mode = Mode.SHOOT;
            shootState = ShootState.A;
            shootTimer.reset();
        }

        switch (mode) {
            case INTAKE:
                runIntake();
                break;
            case SHOOT:
                runShoot();
                break;
            default:
                holdIdle();
                break;
        }

        telemetry.addData("Mode", mode);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Distance (cm)", "%.2f", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Spin", spinServo.getPosition());
        telemetry.addData("CR L/R", "%.2f / %.2f", crLeft.getPower(), crRight.getPower());
        telemetry.update();
    }

    // ================= INTAKE MODE =================
    private void runIntake() {
        // Outputs
        setShooterRPM(0);
        kickServo.setPosition(KICK_IDLE);
        intake.setPower(INTAKE_POWER);
        setCR(INTAKE_CR_POWER, INTAKE_CR_POWER);

        // Detect
        double d = distanceSensor.getDistance(DistanceUnit.CM);
        boolean detected = d <= DISTANCE_CM;

        // Rising edge + cooldown
        if (detected && !lastDetected && detectCooldown.seconds() >= DETECT_COOLDOWN_SEC) {
            if (ballCount < 2) ballCount++;
            detectCooldown.reset();
        }
        lastDetected = detected;

        // Spin staging by count
        if (ballCount == 0) spinServo.setPosition(INTAKE_0);
        else if (ballCount == 1) spinServo.setPosition(INTAKE_1);
        else spinServo.setPosition(INTAKE_2);
    }

    // ================= SHOOT MODE =================
    private void runShoot() {
        // Disable intake during shooting
        intake.setPower(0);

        // Shooter always commanded during shoot
        setShooterRPM(SHOOTER_RPM);

        switch (shootState) {

            // -------- A --------
            case A:
                spinServo.setPosition(SHOOT_A);
                setCR(SHOOT_CR_A_L, SHOOT_CR_A_R);
                kickServo.setPosition(KICK_IDLE);
                shootTimer.reset();
                shootState = ShootState.A_SETTLE;
                break;

            case A_SETTLE:
                spinServo.setPosition(SHOOT_A);
                setCR(SHOOT_CR_A_L, SHOOT_CR_A_R);
                kickServo.setPosition(KICK_IDLE);
                if (shootTimer.seconds() >= SETTLE_SEC) {
                    shootTimer.reset();
                    shootState = ShootState.A_KICK;
                }
                break;

            case A_KICK:
                spinServo.setPosition(SHOOT_A);
                setCR(SHOOT_CR_A_L, SHOOT_CR_A_R);
                kickServo.setPosition(KICK_ACTIVE);
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    shootTimer.reset();
                    shootState = ShootState.A_POST;
                }
                break;

            case A_POST:
                spinServo.setPosition(SHOOT_A);
                setCR(SHOOT_CR_A_L, SHOOT_CR_A_R);
                kickServo.setPosition(KICK_IDLE);
                if (shootTimer.milliseconds() >= POST_MS) {
                    shootState = ShootState.B;
                }
                break;

            // -------- B --------
            case B:
                spinServo.setPosition(SHOOT_B);
                setCR(SHOOT_CR_B_L, SHOOT_CR_B_R);
                kickServo.setPosition(KICK_IDLE);
                shootTimer.reset();
                shootState = ShootState.B_SETTLE;
                break;

            case B_SETTLE:
                spinServo.setPosition(SHOOT_B);
                setCR(SHOOT_CR_B_L, SHOOT_CR_B_R);
                kickServo.setPosition(KICK_IDLE);
                if (shootTimer.seconds() >= SETTLE_SEC) {
                    shootTimer.reset();
                    shootState = ShootState.B_KICK;
                }
                break;

            case B_KICK:
                spinServo.setPosition(SHOOT_B);
                setCR(SHOOT_CR_B_L, SHOOT_CR_B_R);
                kickServo.setPosition(KICK_ACTIVE);
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    shootTimer.reset();
                    shootState = ShootState.B_POST;
                }
                break;

            case B_POST:
                spinServo.setPosition(SHOOT_B);
                setCR(SHOOT_CR_B_L, SHOOT_CR_B_R);
                kickServo.setPosition(KICK_IDLE);
                if (shootTimer.milliseconds() >= POST_MS) {
                    shootState = ShootState.C;
                }
                break;

            // -------- C --------
            case C:
                spinServo.setPosition(SHOOT_C);
                setCR(SHOOT_CR_C_L, SHOOT_CR_C_R);
                kickServo.setPosition(KICK_IDLE);
                shootTimer.reset();
                shootState = ShootState.C_SETTLE;
                break;

            case C_SETTLE:
                spinServo.setPosition(SHOOT_C);
                setCR(SHOOT_CR_C_L, SHOOT_CR_C_R);
                kickServo.setPosition(KICK_IDLE);
                if (shootTimer.seconds() >= SETTLE_SEC) {
                    shootTimer.reset();
                    shootState = ShootState.C_KICK;
                }
                break;

            case C_KICK:
                spinServo.setPosition(SHOOT_C);
                setCR(SHOOT_CR_C_L, SHOOT_CR_C_R);
                kickServo.setPosition(KICK_ACTIVE);
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    shootTimer.reset();
                    shootState = ShootState.C_POST;
                }
                break;

            case C_POST:
                spinServo.setPosition(SHOOT_C);
                setCR(SHOOT_CR_C_L, SHOOT_CR_C_R);
                kickServo.setPosition(KICK_IDLE);
                if (shootTimer.milliseconds() >= POST_MS) {
                    // End: ready to intake
                    spinServo.setPosition(INTAKE_0);
                    setCR(0, 0);
                    setShooterRPM(0);
                    ballCount = 0;     // assume we shot everything
                    mode = Mode.IDLE;  // wait for A or X
                }
                break;
        }
    }

    private void holdIdle() {
        // Safe outputs while waiting
        intake.setPower(0);
        setShooterRPM(0);
        setCR(0, 0);
        kickServo.setPosition(KICK_IDLE);
        // Keep spin at intake-ready
        spinServo.setPosition(INTAKE_0);
    }

    private void setCR(double l, double r) {
        crLeft.setPower(l);
        crRight.setPower(r);
    }

    private void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * TICKS_PER_REV) / 60.0);
    }
}