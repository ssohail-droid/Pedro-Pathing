package pedroPathing.AutoClass.Actuator; // Use your actual package name

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

// The @Config tag enables tuning these variables via the FTC Dashboard
@Config
//@TeleOp(name = "ThreePositionServo", group = "Example")
public class ThreePosServoTest extends OpMode {

    // --- 🎯 DASHBOARD TUNABLE POSITIONS (0.0 to 1.0) ---
    // These static variables can be changed live on the Dashboard!
    public static double POSITION_0_RETRACTED = 0; // Example: Fully retracted
    public static double POSITION_1_MID = 0.45;       // Example: Mid-range position
    public static double POSITION_2_MAX = 0.87;       // Example: Fully extended

    private Servo mechanismServo;
    private double currentTargetPosition = POSITION_0_RETRACTED;

    // Toggle for button press debounce
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadRightPressed = false;

    @Override
    public void init() {
        // Initialize the servo
        // IMPORTANT: Use the exact name configured in the Robot Configuration
        mechanismServo = hardwareMap.get(Servo.class, "spin");

        // Set initial position
        currentTargetPosition = POSITION_0_RETRACTED;
        mechanismServo.setPosition(currentTargetPosition);

        telemetry.addData("Status", "Initialized. Tune positions on Dashboard (0.0 to 1.0).");
        telemetry.addData("Servo", "Ready to move to %.2f", currentTargetPosition);
    }

    @Override
    public void loop() {
        // --- 🎮 GAMEPAD INPUT (Using D-pad on Gamepad 2) ---
        Gamepad gp2 = gamepad2;

        // Button press detection with simple debounce

        // D-PAD UP: Set to Position 2 (Max)
        if (gp2.dpad_up && !dpadUpPressed) {
            currentTargetPosition = POSITION_2_MAX;
            dpadUpPressed = true;
        } else if (!gp2.dpad_up) {
            dpadUpPressed = false;
        }

        // D-PAD DOWN: Set to Position 0 (Retracted)
        if (gp2.dpad_down && !dpadDownPressed) {
            currentTargetPosition = POSITION_0_RETRACTED;
            dpadDownPressed = true;
        } else if (!gp2.dpad_down) {
            dpadDownPressed = false;
        }

        // D-PAD RIGHT: Set to Position 1 (Mid)
        if (gp2.dpad_right && !dpadRightPressed) {
            currentTargetPosition = POSITION_1_MID;
            dpadRightPressed = true;
        } else if (!gp2.dpad_right) {
            dpadRightPressed = false;
        }

        // --- ⚙️ SERVO EXECUTION ---
        mechanismServo.setPosition(currentTargetPosition);

        // --- 📊 DASHBOARD AND TELEMETRY OUTPUT ---
        // Since all three target positions are public static, they automatically appear on the Dashboard
        telemetry.addData("1. Current Target Position", "%.2f", currentTargetPosition);
        telemetry.addData("2. Actual Servo Position", "%.2f", mechanismServo.getPosition());
        telemetry.addData("3. Position 0 (P0)", "%.2f", POSITION_0_RETRACTED);
        telemetry.addData("4. Position 1 (P1)", "%.2f", POSITION_1_MID);
        telemetry.addData("5. Position 2 (P2)", "%.2f", POSITION_2_MAX);
        telemetry.update();
    }
}