package org.firstinspires.ftc.teamcode; // Use your actual team code package

import com.acmerobotics.dashboard.config.Config; // *** IMPORTANT: Import for Dashboard ***
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Dashboard Shooter/Servo Test", group = "Test")
@Config // *** IMPORTANT: Annotate for Dashboard access ***
public class ShooterWithAdjustments extends OpMode {

    // ======= Hardware =======
    private DcMotorEx shooterMotor;
    private Servo adjustServo;

    // ===========================================
    // === DASHBOARD CONFIGURABLE VALUES (public static) ===
    // ===========================================

    // Shooter Control
    public static double SHOOTER_POWER_ON = 0.85;

    // Servo Control
    public static double ADJUST_INCREMENT = 0.01;
    public static double ADJUST_START_POS = 0.50;
    public static double ADJUST_MIN_POS = 0.15;
    public static double ADJUST_MAX_POS = 0.85;

    // ===========================================

    // ======= State Variables (non-static) =======
    private double currentServoPos = ADJUST_START_POS; // Initialize with the start config
    private boolean shooterIsOn = false;

    @Override
    public void init() {
        // 1. Initialize the Shooter Motor
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Shooter ERROR", "Could not find 'shooter' in config!");
        }

        // 2. Initialize the Adjustment Servo
        try {
            adjustServo = hardwareMap.get(Servo.class, "adjust_servo");

            // Set initial position from the configurable value
            currentServoPos = ADJUST_START_POS;
            adjustServo.setPosition(currentServoPos);
        } catch (Exception e) {
            telemetry.addData("Servo ERROR", "Could not find 'adjust_servo' in config!");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        // Re-read the starting position in case the dashboard changed it during init
        currentServoPos = ADJUST_START_POS;
        adjustServo.setPosition(currentServoPos);
    }

    @Override
    public void loop() {

        // ===================================
        // === 1. SHOOTER ON/OFF CONTROL ===
        // ===================================

        // Turn Shooter ON (e.g., Left Bumper)
        if (gamepad1.left_bumper) {
            shooterIsOn = true;
        }

        // Turn Shooter OFF (e.g., Right Bumper)
        if (gamepad1.right_bumper) {
            shooterIsOn = false;
        }

        // Apply the power, using the value dynamically updated by the Dashboard
        if (shooterIsOn) {
            shooterMotor.setPower(SHOOTER_POWER_ON);
        } else {
            shooterMotor.setPower(0.0);
        }

        // ===================================
        // === 2. SERVO ADJUSTMENT CONTROL ===
        // ===================================

        // --- Adjust UP (Uses Right Trigger) ---
        if (gamepad1.right_trigger > 0.1) {
            currentServoPos += ADJUST_INCREMENT;
        }

        // --- Adjust DOWN (Uses Left Trigger) ---
        if (gamepad1.left_trigger > 0.1) {
            currentServoPos -= ADJUST_INCREMENT;
        }

        // --- Limit the Position (Uses configurable min/max) ---
        currentServoPos = Range.clip(currentServoPos, ADJUST_MIN_POS, ADJUST_MAX_POS);

        // Apply the position
        adjustServo.setPosition(currentServoPos);


        // ===================================
        // === 3. TELEMETRY (Driver Station / Dashboard) ===
        // ===================================

        // This telemetry will appear on the FTC Dashboard's 'telemetry' tab
        telemetry.addData("--- DASHBOARD CONFIGS ---", "");
        telemetry.addData("Shooter Power (Config)", SHOOTER_POWER_ON);
        telemetry.addData("Servo Increment (Config)", ADJUST_INCREMENT);
        telemetry.addData("--- ROBOT STATUS ---", "");
        telemetry.addData("Shooter Status", shooterIsOn ? "RUNNING" : "STOPPED");
        telemetry.addData("Current Servo Position", "%.4f", currentServoPos); // High precision for tuning
        telemetry.update();
    }
}