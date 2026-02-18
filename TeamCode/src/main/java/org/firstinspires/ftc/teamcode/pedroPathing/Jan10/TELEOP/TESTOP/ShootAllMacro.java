package org.firstinspires.ftc.teamcode.pedroPathing.Jan10.TELEOP.TESTOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "ShootAll Macro (X)", group = "Main")
@Config
public class ShootAllMacro extends OpMode {

    // ===== HARDWARE =====
    private DcMotorEx shooter;
    private Servo spinServo;
    private Servo kickServo;
    private CRServo crLeft, crRight;

    // ===== DASHBOARD TUNABLES =====
    public static double SHOOTER_RPM = 0;
    public static double TICKS_PER_REV = 28.0;

    // Spin positions
    public static double SPIN_A = 0.56;
    public static double SPIN_B = 0.32;
    public static double SPIN_C = 0.04;
    public static double SPIN_READY = 0.18; // ✅ intake-ready end position

    public static double SETTLE_TIME_SEC = 2.0;

    // Kicker
    public static double KICKER_IDLE = 1.0;
    public static double KICKER_ACTIVE = 0.7;
    public static double KICK_HOLD_MS = 1000;
    public static double POST_KICK_COOLDOWN_MS = 500;

    // CR servo profiles (USED correctly now)
    public static double CR_A_L = 1.0, CR_A_R = -1.0;
    public static double CR_B_L = 1.0, CR_B_R = -1.0;
    public static double CR_C_L = 1.0, CR_C_R = -1.0;

    // PWM range for 5-turn servo
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    // ===== STATE MACHINE =====
    private enum State {
        IDLE,
        INIT_A, SETTLE_A, KICK_A, POST_A,
        MOVE_B, SETTLE_B, KICK_B, POST_B,
        MOVE_C, SETTLE_C, KICK_C, POST_C,
        DONE
    }

    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean lastX = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        spinServo = hardwareMap.get(Servo.class, "spin");
        if (spinServo instanceof PwmControl) {
            ((PwmControl) spinServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        }

        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        kickServo.setPosition(KICKER_IDLE);

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        crLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo.setPosition(SPIN_READY);
        setCR(0, 0);
        setShooterRPM(0);
    }

    @Override
    public void loop() {
        boolean xPressed = gamepad1.x && !lastX;
        lastX = gamepad1.x;

        if (xPressed && (state == State.IDLE || state == State.DONE)) {
            transition(State.INIT_A);
        }

        switch (state) {

            case IDLE:
                spinServo.setPosition(SPIN_READY);
                setShooterRPM(0);
                setCR(0, 0);
                break;

            case INIT_A:
                spinServo.setPosition(SPIN_A);
                setShooterRPM(SHOOTER_RPM);
                setCR(CR_A_L, CR_A_R);
                transition(State.SETTLE_A);
                break;

            case SETTLE_A:
                if (timer.seconds() >= SETTLE_TIME_SEC) transition(State.KICK_A);
                break;

            case KICK_A:
                kickServo.setPosition(KICKER_ACTIVE);
                if (timer.milliseconds() >= KICK_HOLD_MS) {
                    kickServo.setPosition(KICKER_IDLE);
                    transition(State.POST_A);
                }
                break;

            case POST_A:
                if (timer.milliseconds() >= POST_KICK_COOLDOWN_MS) transition(State.MOVE_B);
                break;

            case MOVE_B:
                spinServo.setPosition(SPIN_B);
                setCR(CR_B_L, CR_B_R);
                transition(State.SETTLE_B);
                break;

            case SETTLE_B:
                if (timer.seconds() >= SETTLE_TIME_SEC) transition(State.KICK_B);
                break;

            case KICK_B:
                kickServo.setPosition(KICKER_ACTIVE);
                if (timer.milliseconds() >= KICK_HOLD_MS) {
                    kickServo.setPosition(KICKER_IDLE);
                    transition(State.POST_B);
                }
                break;

            case POST_B:
                if (timer.milliseconds() >= POST_KICK_COOLDOWN_MS) transition(State.MOVE_C);
                break;

            case MOVE_C:
                spinServo.setPosition(SPIN_C);
                setCR(CR_C_L, CR_C_R);
                transition(State.SETTLE_C);
                break;

            case SETTLE_C:
                if (timer.seconds() >= SETTLE_TIME_SEC) transition(State.KICK_C);
                break;

            case KICK_C:
                kickServo.setPosition(KICKER_ACTIVE);
                if (timer.milliseconds() >= KICK_HOLD_MS) {
                    kickServo.setPosition(KICKER_IDLE);
                    transition(State.POST_C);
                }
                break;

            case POST_C:
                if (timer.milliseconds() >= POST_KICK_COOLDOWN_MS) transition(State.DONE);
                break;

            case DONE:
                spinServo.setPosition(SPIN_READY); // ✅ END READY TO INTAKE
                setShooterRPM(0);
                setCR(0, 0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Spin Pos", spinServo.getPosition());
        telemetry.addData("Shooter RPM", SHOOTER_RPM);
        telemetry.update();
    }

    // ===== HELPERS =====
    private void transition(State next) {
        state = next;
        timer.reset();
    }

    private void setCR(double l, double r) {
        crLeft.setPower(l);
        crRight.setPower(r);
    }

    private void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * TICKS_PER_REV) / 60.0);
    }
}
