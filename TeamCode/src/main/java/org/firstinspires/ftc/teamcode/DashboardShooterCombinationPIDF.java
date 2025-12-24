package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "Full Shooter System v3 - PIDF", group = "Main")
@Config
public class DashboardShooterCombinationPIDF extends OpMode {

    // ======= ⚙️ HARDWARE =======
    private DcMotorEx shooterMotor;
    private Servo kick_servo;
    private Servo spin_servo;
    private FtcDashboard dashboard;

    // ======= 📊 DASHBOARD TUNABLE VALUES =======

    // Shooter PIDF Settings
    public static double TARGET_RPM = 2450;
    public static double TICKS_PER_REV = 28.0;
    public static double P = 65.0;
    public static double I = 2.0;
    public static double D = 0.00009;
    public static double F = 13.0;

    // Kicker Settings
    public static double KICKER_POS_ACTIVE = 0.7;
    public static double KICKER_POS_IDLE = 1.0;
    public static double KICKER_DELAY_MS = 1000;

    // Spin Servo Positions
    public static double SPIN_RETRACTED = 0.0;
    public static double SPIN_MID = 0.45;
    public static double SPIN_MAX = 0.87;
    public static double SPIN_CUSTOM_1 = 0.68;
    public static double SPIN_CUSTOM_2 = 0.2;
    public static double SPIN_CUSTOM_3 = 1.0;

    // ======= 🔄 STATE VARIABLES =======
    private boolean shooterIsOn = false;
    private double currentSpinTarget = SPIN_RETRACTED;

    private boolean kickerIsActive = false;
    private ElapsedTime kickerTimer = new ElapsedTime();

    private ElapsedTime buttonTimer = new ElapsedTime();
    private static final double DEBOUNCE_TIME = 0.25;
    private boolean lastToggleButtonState = false;

    @Override
    public void init() {
        // Initialize Dashboard Telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // --- Shooter ---
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) { telemetry.addData("Error", "Shooter not found"); }

        // --- Kicker ---
        try {
            kick_servo = hardwareMap.get(Servo.class, "kick_servo");
            kick_servo.setPosition(KICKER_POS_IDLE);
        } catch (Exception e) { telemetry.addData("Error", "kick_servo not found"); }

        // --- Spin ---
        try {
            spin_servo = hardwareMap.get(Servo.class, "spin");
            spin_servo.setPosition(SPIN_RETRACTED);
        } catch (Exception e) { telemetry.addData("Error", "spin servo not found"); }

        buttonTimer.reset();
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- 1. SHOOTER & PIDF LOGIC ---
        if (gamepad1.a && !lastToggleButtonState && buttonTimer.seconds() > DEBOUNCE_TIME) {
            shooterIsOn = !shooterIsOn;
            buttonTimer.reset();
        }
        lastToggleButtonState = gamepad1.a;

        // Apply Live PIDF Coefficients from Dashboard
        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);

        double targetTPS = shooterIsOn ? (TARGET_RPM * TICKS_PER_REV) / 60.0 : 0.0;
        shooterMotor.setVelocity(targetTPS);

        // --- 2. KICKER LOGIC ---
        if (gamepad1.right_trigger > 0.5 && !kickerIsActive) {
            kickerIsActive = true;
            kickerTimer.reset();
            kick_servo.setPosition(KICKER_POS_ACTIVE);
        }
        if (kickerIsActive && kickerTimer.milliseconds() > KICKER_DELAY_MS) {
            kick_servo.setPosition(KICKER_POS_IDLE);
            kickerIsActive = false;
        }
        if (gamepad1.left_trigger > 0.5) {
            kick_servo.setPosition(KICKER_POS_IDLE);
            kickerIsActive = false;
        }

        // --- 3. SPIN SERVO LOGIC ---
        if (gamepad1.dpad_up)    currentSpinTarget = SPIN_MAX;
        if (gamepad1.dpad_down)  currentSpinTarget = SPIN_RETRACTED;
        if (gamepad1.dpad_right) currentSpinTarget = SPIN_MID;
        if (gamepad1.dpad_left)  currentSpinTarget = SPIN_CUSTOM_1;
        if (gamepad1.x)          currentSpinTarget = SPIN_CUSTOM_2;
        if (gamepad1.y)          currentSpinTarget = SPIN_CUSTOM_3;
        spin_servo.setPosition(currentSpinTarget);

        // --- 4. TELEMETRY & GRAPHING ---
        double currentTPS = shooterMotor.getVelocity();
        double actualRPM = (currentTPS * 60.0) / TICKS_PER_REV;
        double errorRPM = (shooterIsOn ? TARGET_RPM : 0) - actualRPM;

        telemetry.addData("Target RPM", shooterIsOn ? TARGET_RPM : 0);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("Error RPM", errorRPM);
        telemetry.addData("Kicker State", kickerIsActive ? "KICKING" : "IDLE");
        telemetry.addData("Spin Pos", currentSpinTarget);
        telemetry.update();
    }
}