package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Shooter + Color Sequence", group = "Main")
@Config
public class DashboardShooterColor extends OpMode {

    //// ======= ⚙️ HARDWARE =======
    private DcMotorEx shooterMotor, intake;
    private Servo kick_servo, spin_servo;
    private NormalizedColorSensor colorSensorLeft, colorSensorRight;
    private FtcDashboard dashboard;

    // ======= 📊 DASHBOARD TUNABLE VALUES =======
    public static double INTAKE_POWER = 0.8;
    public static double DISTANCE_LIMIT_CM = 5.0;
    public static double COLOR_COOLDOWN = 1.5; // Seconds to wait before next detection

    // Shooter PIDF
    public static double TARGET_RPM = 2450;
    public static double TICKS_PER_REV = 28.0;
    public static double P = 65.0, I = 2.0, D = 0.00009, F = 13.0;

    // Kicker & Spin Positions
    public static double KICKER_POS_ACTIVE = 0.7, KICKER_POS_IDLE = 1.0, KICKER_DELAY_MS = 1000;
    public static double SPIN_RETRACTED = 0.205, SPIN_MID = 0.5, SPIN_MAX = 0.64;
    public static double SPIN_CUSTOM_1 = 0.71, SPIN_CUSTOM_2 = 1.0, SPIN_CUSTOM_3 = 0.86;

    // ======= 🔄 STATE VARIABLES =======
    private boolean shooterIsOn = false;
    private boolean intakeIsOn = false;
    private boolean sequenceActive = false;
    private int sequenceStep = 0; // 0: Retracted, 1: Mid, 2: Max

    private int purpleCount = 0;
    private int greenCount = 0;

    private double currentSpinTarget = SPIN_RETRACTED;
    private boolean kickerIsActive = false;

    private ElapsedTime kickerTimer = new ElapsedTime();
    private ElapsedTime colorCooldownTimer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime();
    private static final double DEBOUNCE_TIME = 0.3;

    private boolean lastA = false, lastB = false, lastY = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) { telemetry.addData("Error", "Shooter not found"); }

        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        } catch (Exception e) { telemetry.addData("Error", "Intake not found"); }

        try {
            kick_servo = hardwareMap.get(Servo.class, "kick_servo");
            kick_servo.setPosition(KICKER_POS_IDLE);
        } catch (Exception e) { telemetry.addData("Error", "Kicker not found"); }

        try {
            spin_servo = hardwareMap.get(Servo.class, "spin");
            if (spin_servo instanceof PwmControl) {
                ((PwmControl) spin_servo).setPwmRange(new PwmControl.PwmRange(800, 2200));
            }
            spin_servo.setPosition(SPIN_RETRACTED);
        } catch (Exception e) { telemetry.addData("Error", "Spin not found"); }

        try {
            colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_left");
            colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_right");
        } catch (Exception e) { telemetry.addData("Error", "Color sensors not found"); }

        colorCooldownTimer.reset();
        debounceTimer.reset();
    }

    @Override
    public void loop() {
        // --- 1. TOGGLE CONTROLS ---
        if (gamepad1.a && !lastA && debounceTimer.seconds() > DEBOUNCE_TIME) {
            shooterIsOn = !shooterIsOn;
            debounceTimer.reset();
        }
        if (gamepad1.b && !lastB && debounceTimer.seconds() > DEBOUNCE_TIME) {
            intakeIsOn = !intakeIsOn;
            debounceTimer.reset();
        }
        if (gamepad1.y && !lastY && debounceTimer.seconds() > DEBOUNCE_TIME) {
            sequenceActive = !sequenceActive;
            if (sequenceActive) {
                sequenceStep = 0; // Reset sequence to start
                intakeIsOn = true;
            }
            debounceTimer.reset();
        }
        lastA = gamepad1.a; lastB = gamepad1.b; lastY = gamepad1.y;

        // --- 2. SEQUENCE LOGIC (THE REQUESTED FEATURE) ---
        if (sequenceActive) {
            // Update target based on step
            if (sequenceStep == 0) currentSpinTarget = SPIN_RETRACTED;
            else if (sequenceStep == 1) currentSpinTarget = SPIN_MID;
            else if (sequenceStep == 2) currentSpinTarget = SPIN_MAX;
            else {
                sequenceActive = false; // Finished 3 steps
                intakeIsOn = false;
            }

            // Check Sensors
            if (sequenceActive && colorCooldownTimer.seconds() > COLOR_COOLDOWN) {
                String detected = checkColorSensors();
                if (!detected.equals("NONE")) {
                    if (detected.equals("PURPLE")) purpleCount++;
                    else if (detected.equals("GREEN")) greenCount++;

                    sequenceStep++; // Move to next intake position
                    colorCooldownTimer.reset();
                }
            }
        } else {
            // MANUAL SPIN CONTROL (Only if sequence is OFF)
            if (gamepad1.dpad_up)    currentSpinTarget = SPIN_MAX;
            if (gamepad1.dpad_down)  currentSpinTarget = SPIN_RETRACTED;
            if (gamepad1.dpad_right) currentSpinTarget = SPIN_MID;
            if (gamepad1.dpad_left)  currentSpinTarget = SPIN_CUSTOM_1;
            if (gamepad1.x)          currentSpinTarget = SPIN_CUSTOM_2;
            // Note: Gamepad1.y is now the Sequence Toggle
        }

        // --- 3. EXECUTE HARDWARE COMMANDS ---
        // Shooter
        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
        shooterMotor.setVelocity(shooterIsOn ? (TARGET_RPM * TICKS_PER_REV) / 60.0 : 0.0);

        // Intake
        intake.setPower(intakeIsOn ? INTAKE_POWER : 0);

        // Kicker
        if (gamepad1.right_trigger > 0.5 && !kickerIsActive) {
            kickerIsActive = true;
            kickerTimer.reset();
            kick_servo.setPosition(KICKER_POS_ACTIVE);
        }
        if (kickerIsActive && kickerTimer.milliseconds() > KICKER_DELAY_MS) {
            kick_servo.setPosition(KICKER_POS_IDLE);
            kickerIsActive = false;
        }

        spin_servo.setPosition(currentSpinTarget);

        // --- 4. TELEMETRY ---
        telemetry.addData("MODE", sequenceActive ? "AUTO-SEQUENCE" : "MANUAL");
        telemetry.addData("Sequence Step", sequenceStep);
        telemetry.addData("Purple Count", purpleCount);
        telemetry.addData("Green Count", greenCount);
        telemetry.addLine("--- Hardware ---");
        telemetry.addData("Shooter", shooterIsOn ? "ON" : "OFF");
        telemetry.addData("Intake", intakeIsOn ? "ON" : "OFF");
        telemetry.addData("Spin Pos", currentSpinTarget);
        telemetry.update();
    }

    private String checkColorSensors() {
        if (isColor(colorSensorLeft, "PURPLE") || isColor(colorSensorRight, "PURPLE")) return "PURPLE";
        if (isColor(colorSensorLeft, "GREEN") || isColor(colorSensorRight, "GREEN")) return "GREEN";
        return "NONE";
    }

    private boolean isColor(NormalizedColorSensor sensor, String colorName) {
        if (sensor == null) return false;

        float[] hsv = new float[3];
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsv);
        double dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);

        if (dist > DISTANCE_LIMIT_CM) return false;

        if (colorName.equals("PURPLE")) return (hsv[0] >= 200 && hsv[0] <= 300);
        if (colorName.equals("GREEN")) return (hsv[0] >= 100 && hsv[0] <= 190);

        return false;
    }
}