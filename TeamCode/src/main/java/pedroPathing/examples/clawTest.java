package pedroPathing.examples;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="claw test", group="Iterative OpMode")
public class clawTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo gripperServo = null;
    private Servo flipWrist = null;
    // Variables to track toggle states
    boolean gripperToggleState = false;
    boolean flipWristToggleState = false;
    // Variables to track button press history
    boolean leftBumperPrev = false;
    boolean rightBumperPrev = false;

    @Override
    public void init() {
        gripperServo = hardwareMap.get(Servo.class, "claw");
        flipWrist = hardwareMap.get(Servo.class, "wrist");

        // Display initialization status
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Handle gripper toggle
        if (gamepad2.left_bumper && !leftBumperPrev) { // Button pressed
            gripperToggleState = !gripperToggleState; // Toggle state
            gripperServo.setPosition(gripperToggleState ? 0.15 : 0);
        }
        leftBumperPrev = gamepad2.left_bumper; // Update previous state

        // Handle flip wrist toggle
        if (gamepad2.right_bumper && !rightBumperPrev) { // Button pressed
            flipWristToggleState = !flipWristToggleState; // Toggle state
            flipWrist.setPosition(flipWristToggleState ? 0 : 0.7);
        }
        rightBumperPrev = gamepad2.right_bumper; // Update previous state

        // Telemetry for servos
        telemetry.addData("Gripper Servo Position", gripperServo.getPosition());
        telemetry.addData("Flip Wrist Servo Position", flipWrist.getPosition());
        telemetry.addData("Gripper Toggle State", gripperToggleState);
        telemetry.addData("Flip Wrist Toggle State", flipWristToggleState);

        // Display runtime
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    private void debounceDelay() {
        try {
            sleep(300); // Debounce delay
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void stop() {
        // Stop all operations when the OpMode is stopped
    }
}