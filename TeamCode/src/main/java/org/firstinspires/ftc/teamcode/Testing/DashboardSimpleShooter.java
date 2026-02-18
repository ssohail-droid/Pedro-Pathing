package org.firstinspires.ftc.teamcode.Testing; // Ensure this matches your team's package structure

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "Dashboard RPM Shooter", group = "Test")
@Config // Enables FTC Dashboard access
public class DashboardSimpleShooter extends OpMode {

    // ======= Hardware =======
    private DcMotorEx shooterMotor;

    // ===========================================
    // === DASHBOARD CONFIGURABLE VALUES (public static) ===
    // ===========================================

    // Target RPM for the shooter (Adjustable via Dashboard)
    public static double TARGET_RPM = 2450;

    // Motor Ticks Per Revolution (CRITICAL: MUST MATCH YOUR MOTOR ENCODER!)
    // Example: 28.0 (for a REV Core Hex or similar motor)
    private static final double TICKS_PER_REV = 28.0;

    // Optional: PID Debounce time for cleaner button toggling
    private static final double BUTTON_DEBOUNCE_TIME = 0.25; // 250 ms delay

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

            // Set motor direction to REVERSE (as requested)
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Set motor mode to use PID velocity control
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Shooter Status", "Motor Initialized (RPM Control).");
        } catch (Exception e) {
            telemetry.addData("Shooter ERROR", "Could not find 'shooter' in config!");
        }

        buttonTimer.reset();
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===================================
        // === 1. SHOOTER ON/OFF TOGGLE ===
        // ===================================

        // Use A button to toggle the shooter state
        boolean currentToggleButtonState = gamepad1.a;

        // Rising Edge Detection: Only flip state when the button is initially pressed AND the timer has passed
        if (currentToggleButtonState && !lastToggleButtonState && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            shooterIsOn = !shooterIsOn; // Flip the state (ON -> OFF or OFF -> ON)
            buttonTimer.reset();        // Reset the timer for the next press
        }
        lastToggleButtonState = currentToggleButtonState;


        // ==========================================================
        // === 2. APPLY VELOCITY (READS TARGET_RPM IN REAL-TIME) ===
        // ==========================================================

        double targetTicksPerSecond = 0.0;

        if (shooterIsOn) {
            // FIX FOR DASHBOARD REAL-TIME TUNING: Calculate TPS EVERY LOOP cycle
            // Ticks/Second = (RPM * Ticks/Rev) / 60 seconds
            targetTicksPerSecond = (TARGET_RPM * TICKS_PER_REV) / 60.0;
        }

        // Apply the velocity. This is 0.0 if shooterIsOn is false.
        shooterMotor.setVelocity(targetTicksPerSecond);


        // ===================================
        // === 3. TELEMETRY (Driver Station / Dashboard) ===
        // ===================================

        // Calculate the actual RPM the motor is running at for feedback
        double currentRPM = (shooterMotor.getVelocity() * 60.0) / TICKS_PER_REV;

        telemetry.addData("--- CONFIG ---", "");
        telemetry.addData("Target RPM (Dashboard)", TARGET_RPM);
        telemetry.addData("Ticks/Rev (Constant)", TICKS_PER_REV);
        telemetry.addData("--- STATUS ---", "");
        telemetry.addData("TOGGLE Button", "Gamepad 1 A");
        telemetry.addData("Shooter Status", shooterIsOn ? "RUNNING" : "STOPPED");
        telemetry.addData("Actual RPM", "%.1f", currentRPM);
        telemetry.addData("Target TPS", "%.1f", targetTicksPerSecond);
        telemetry.update();
    }
}