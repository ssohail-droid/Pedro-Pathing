package org.firstinspires.ftc.teamcode.pedroPathing.AutoClass.Actuator; // Your team code package

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config; // Needed for Dashboard tuning
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime; // For the cooldown timer
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// The @Config tag enables tuning these variables via the FTC Dashboard
@Config
//@TeleOp(name = "ColorTriggeredAlternatingServo", group = "Custom")
public class test extends LinearOpMode {

    // --- 🎯 DASHBOARD TUNABLE POSITIONS (0.0 to 1.0) ---
    public static double POSITION_0_RETRACTED = 0.205;   // Lowest Position
    public static double POSITION_1_MID = 0.50;      // Mid-range position
    public static double POSITION_2_MAX = 0.64;      // Highest Position

    // Array to hold the positions for easy indexing
    private final double[] servoPositions = {POSITION_0_RETRACTED, POSITION_1_MID, POSITION_2_MAX};

    // ---  SERVO VARIABLES ---
    private Servo mechanismServo;
    private int currentPositionIndex = 0;
    private int direction = 1;

    // ---  TIMING VARIABLES ---
    private ElapsedTime cooldownTimer = new ElapsedTime();
    private static final double COOLDOWN_SECONDS = 3.0;

    // ---  COLOR SENSOR VARIABLES ---
    NormalizedColorSensor colorSensorLeft;
    NormalizedColorSensor colorSensorRight;
    private static final double DISTANCE_LIMIT_CM = 5.0;

    @Override
    public void runOpMode() {
        // --- HARDWARE MAPPING ---
        try {
            // Servo initialization (using "spin" from the first code)
            mechanismServo = hardwareMap.get(Servo.class, "spin");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not find Servo 'spin'. Check config.");
        }

        try {
            // Color Sensor mapping (from the second code)
            colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_left");
            colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_right");
            // Optional: Turn on the white LED to improve reliability.
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not find one or more Color Sensors. Check config.");
        }

        // --- INITIALIZATION ---
        // Set initial servo position
        if (mechanismServo != null) {
            mechanismServo.setPosition(servoPositions[currentPositionIndex]);
        }

        cooldownTimer.reset(); // Start the cooldown timer

        telemetry.addData("Status", "Initialized. Ready to move.");
        telemetry.addData("Initial Servo Position", String.format("%.2f (P%d)", servoPositions[currentPositionIndex], currentPositionIndex));
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check if a color has been detected AND the cooldown is over
            boolean isObjectDetected = processSensorData(colorSensorLeft, "LEFT") ||
                    processSensorData(colorSensorRight, "RIGHT");

            // --- SERVO CONTROL LOGIC ---
            if (isObjectDetected && cooldownTimer.seconds() >= COOLDOWN_SECONDS) {

                // 1. Calculate the next position index
                int nextPositionIndex = currentPositionIndex + direction;

                // 2. Check for boundary conditions and reverse direction if needed
                if (nextPositionIndex >= servoPositions.length) { // Reached Max Position (P2)
                    direction = -1; // Change to reverse direction
                    nextPositionIndex = servoPositions.length - 2; // Move from P2 back to P1
                } else if (nextPositionIndex < 0) { // Reached Min Position (P0)
                    direction = 1; // Change to forward direction
                    nextPositionIndex = 1; // Move from P0 back to P1
                }

                // 3. Update position and move servo
                currentPositionIndex = nextPositionIndex;
                mechanismServo.setPosition(servoPositions[currentPositionIndex]);

                // 4. Reset cooldown timer
                cooldownTimer.reset();

                telemetry.addLine("*** MOVEMENT TRIGGERED ***");
                telemetry.addData("Cooldown Remaining", "RESET");
            }

            // --- TELEMETRY ---
            telemetry.addData("Current Servo Position (P)", String.format("%.2f (P%d)", servoPositions[currentPositionIndex], currentPositionIndex));
            telemetry.addData("Actual Servo Position", String.format("%.2f", mechanismServo.getPosition()));
            telemetry.addData("Movement Direction", direction == 1 ? "P0->P2" : "P2->P0");
            telemetry.addData("Cooldown Remaining (s)", String.format("%.1f", Math.max(0, COOLDOWN_SECONDS - cooldownTimer.seconds())));

            telemetry.update();
        }
    }


    private boolean processSensorData(NormalizedColorSensor sensor, String label) {
        if (sensor == null) return false;

        NormalizedRGBA colors = sensor.getNormalizedColors();
        double distanceCM = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        String detectedColor = "UNKNOWN";
        double hue = (double) hsvValues[0];
        boolean colorMatch = false;

        // Check if object is close enough AND has a valid Hue
        if (distanceCM < DISTANCE_LIMIT_CM) {
            // --- Purple Check: Hue roughly between 200 and 300 degrees ---
            if (hue >= 200 && hue <= 300) {
                detectedColor = "**PURPLE**";
                colorMatch = true;
            }
            // --- Green Check: Hue roughly between 100 and 190 degrees ---
            else if (hue >= 100 && hue <= 190) {
                detectedColor = "**GREEN**";
                colorMatch = true;
            }
        } else {
            detectedColor = "OUT OF RANGE (> " + String.format("%.1f", DISTANCE_LIMIT_CM) + " cm)";
        }

        // --- Display Telemetry ---
        telemetry.addLine("");
        telemetry.addLine("*** " + label + " Sensor Data ***");
        telemetry.addData(label + " Color", detectedColor + (colorMatch ? " (TRIGGER)" : ""));
        telemetry.addData(label + " Distance (CM)", String.format("%.1f", distanceCM));
        telemetry.addData(label + " Hue", String.format("%.0f", hue));

        return colorMatch;
    }
}