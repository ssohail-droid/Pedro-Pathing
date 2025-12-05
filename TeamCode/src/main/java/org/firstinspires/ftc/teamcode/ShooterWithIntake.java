package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Full Shooter System With Intake", group = "Main")
@Config
public class ShooterWithIntake extends OpMode {

    // ======= ⚙️ HARDWARE =======
    private DcMotorEx shooterMotor;
    private DcMotorEx intake; // Added intake motor
    private Servo kick_servo;
    private Servo spin_servo;
    private FtcDashboard dashboard;

    // ======= 📊 DASHBOARD TUNABLE VALUES =======
    public static double INTAKE_POWER = 0.8; // Power for the intake

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
    public static double SPIN_RETRACTED = 0.205;
    public static double SPIN_MID = 0.5;
    public static double SPIN_MAX = 0.64;
    public static double SPIN_CUSTOM_1 = 0.71;
    public static double SPIN_CUSTOM_2 = 1.0;
    public static double SPIN_CUSTOM_3 = 0.86;

    // ======= 🔄 STATE VARIABLES =======
    private boolean shooterIsOn = false;
    private boolean intakeIsOn = false; // Added intake state
    private double currentSpinTarget = SPIN_RETRACTED;

    private boolean kickerIsActive = false;
    private ElapsedTime kickerTimer = new ElapsedTime();

    private ElapsedTime buttonTimer = new ElapsedTime();
    private ElapsedTime intakeButtonTimer = new ElapsedTime(); // Timer for B button debounce
    private static final double DEBOUNCE_TIME = 0.25;
    private boolean lastToggleButtonState = false;
    private boolean lastIntakeButtonState = false; // Last state of B button

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // --- Shooter ---
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) { telemetry.addData("Error", "Shooter not found"); }

        // --- Intake ---
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) { telemetry.addData("Error", "Intake not found"); }

        // --- Kicker ---
        try {
            kick_servo = hardwareMap.get(Servo.class, "kick_servo");
            kick_servo.setPosition(KICKER_POS_IDLE);
        } catch (Exception e) { telemetry.addData("Error", "kick_servo not found"); }

        // --- Spin (HSR-M9382TH) ---
        try {
            spin_servo = hardwareMap.get(Servo.class, "spin");
            if (spin_servo instanceof PwmControl) {
                ((PwmControl) spin_servo).setPwmRange(new PwmControl.PwmRange(800, 2200));
            }
            spin_servo.setPosition(SPIN_RETRACTED);
        } catch (Exception e) { telemetry.addData("Error", "spin servo not found"); }

        buttonTimer.reset();
        intakeButtonTimer.reset();
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

        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
        double targetTPS = shooterIsOn ? (TARGET_RPM * TICKS_PER_REV) / 60.0 : 0.0;
        shooterMotor.setVelocity(targetTPS);

        // --- 2. INTAKE LOGIC (Gamepad1 B) ---
        if (gamepad1.b && !lastIntakeButtonState && intakeButtonTimer.seconds() > DEBOUNCE_TIME) {
            intakeIsOn = !intakeIsOn;
            intakeButtonTimer.reset();
        }
        lastIntakeButtonState = gamepad1.b;

        if (intakeIsOn) {
            intake.setPower(INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        // --- 3. KICKER LOGIC ---
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

        // --- 4. SPIN SERVO LOGIC ---
        if (gamepad1.dpad_up)    currentSpinTarget = SPIN_MAX;
        if (gamepad1.dpad_down)  currentSpinTarget = SPIN_RETRACTED;
        if (gamepad1.dpad_right) currentSpinTarget = SPIN_MID;
        if (gamepad1.dpad_left)  currentSpinTarget = SPIN_CUSTOM_1;
        if (gamepad1.x)          currentSpinTarget = SPIN_CUSTOM_2;
        if (gamepad1.y)          currentSpinTarget = SPIN_CUSTOM_3;

        spin_servo.setPosition(currentSpinTarget);

        // --- 5. TELEMETRY ---
        telemetry.addData("Shooter", shooterIsOn ? "ON" : "OFF");
        telemetry.addData("Intake", intakeIsOn ? "ON" : "OFF");
        telemetry.addData("Actual RPM", (shooterMotor.getVelocity() * 60.0) / TICKS_PER_REV);
        telemetry.update();
    }
}