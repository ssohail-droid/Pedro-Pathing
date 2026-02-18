package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo; // Required for Servo
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "Dashboard RPM Shooter + Kicker", group = "Test")
@Config
public class ShooterWithKicker extends OpMode {

    // ======= Hardware =======
    private DcMotorEx shooterMotor;
    private Servo kick_servo; // New Servo

    // ===========================================
    // === DASHBOARD CONFIGURABLE VALUES (public static) ===
    // ===========================================
    public static double TARGET_RPM = 2450;
    private static final double TICKS_PER_REV = 28.0;
    private static final double BUTTON_DEBOUNCE_TIME = 0.25;

    // Kicker Positions (Can be tuned via Dashboard)
    public static double KICKER_POS_RIGHT = 1.0;
    public static double KICKER_POS_LEFT = 0.0;
    // ===========================================

    // ======= State Variables =======
    private boolean shooterIsOn = false;
    private ElapsedTime buttonTimer = new ElapsedTime();
    private boolean lastToggleButtonState = false;

    @Override
    public void init() {
        // 1. Initialize the Shooter Motor
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Shooter Status", "Motor Initialized.");
        } catch (Exception e) {
            telemetry.addData("Shooter ERROR", "Could not find 'shooter' in config!");
        }

        // 2. Initialize the Kicker Servo
        try {
            kick_servo = hardwareMap.get(Servo.class, "kick_servo");
            // Set a default starting position
            kick_servo.setPosition(KICKER_POS_LEFT);
            telemetry.addData("Servo Status", "kick_servo Initialized.");
        } catch (Exception e) {
            telemetry.addData("Servo ERROR", "Could not find 'kick_servo' in config!");
        }

        buttonTimer.reset();
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===================================
        // === 1. SHOOTER ON/OFF TOGGLE ===
        // ===================================
        boolean currentToggleButtonState = gamepad1.a;
        if (currentToggleButtonState && !lastToggleButtonState && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            shooterIsOn = !shooterIsOn;
            buttonTimer.reset();
        }
        lastToggleButtonState = currentToggleButtonState;


        // ==========================================================
        // === 2. APPLY VELOCITY (READS TARGET_RPM IN REAL-TIME) ===
        // ==========================================================
        double targetTicksPerSecond = 0.0;
        if (shooterIsOn) {
            targetTicksPerSecond = (TARGET_RPM * TICKS_PER_REV) / 60.0;
        }
        shooterMotor.setVelocity(targetTicksPerSecond);


        // ===================================
        // === 3. KICKER SERVO CONTROL ===
        // ===================================
        // Right Trigger sets servo to 1.0
        if (gamepad1.right_trigger > 0.5) {
            kick_servo.setPosition(KICKER_POS_RIGHT);
        }
        // Left Trigger sets servo to 0.0
        else if (gamepad1.left_trigger > 0.5) {
            kick_servo.setPosition(KICKER_POS_LEFT);
        }


        // ===================================
        // === 4. TELEMETRY ===
        // ===================================
        double currentRPM = (shooterMotor.getVelocity() * 60.0) / TICKS_PER_REV;

        telemetry.addData("Shooter Status", shooterIsOn ? "RUNNING" : "STOPPED");
        telemetry.addData("Actual RPM", "%.1f", currentRPM);
        telemetry.addData("Servo Position", kick_servo.getPosition());
        telemetry.update();
    }
}