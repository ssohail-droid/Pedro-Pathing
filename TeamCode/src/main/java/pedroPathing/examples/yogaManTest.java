//package pedroPathing.examples;
//
//import static java.lang.Thread.sleep;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Disabled
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Yoga man test", group="Iterative OpMode")
//public class yogaManTest extends OpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor sliderMotor;
//
//    // Slider movement limits in encoder ticks
//    private static final int SLIDER_MIN_TICKS = 0;     // Minimum position
//    private static final int SLIDER_MAX_TICKS = 2250;  // Maximum position
//    private static final double SLIDER_POWER = 1;    // Slider movement power
//    // PID control parameters (Proportional control only for simplicity)
//    private static final double Kp = 0.01;  // Proportional gain
//    private static final double HOLD_POWER = 0.05; // Small power for maintaining position
//    private int targetPosition = 0;
//    // Direction constants
//    private static final int CW = 1;   // Clockwise direction
//    private static final int CCW = -1; // Counterclockwise direction
//
//
//    @Override
//    public void init() {
//        // Initialize the slider motor
//        sliderMotor = hardwareMap.get(DcMotor.class, "lift");
//        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void start() {
//        runtime.reset();
//
//    }
//
//    @Override
//    public void loop() {
//
//        // Control the slider motor for holding position
//        if (gamepad2.a) {
//            // Move up if below the maximum limit
//            targetPosition = Math.min(sliderMotor.getCurrentPosition() + 10, SLIDER_MAX_TICKS);
//        } else if (gamepad2.b) {
//            // Move down if above the minimum limit
//            targetPosition = Math.max(sliderMotor.getCurrentPosition() -10, SLIDER_MIN_TICKS);
//        }
//
//
//        // P-Control: Calculate error between current and target position
//        int currentPosition = sliderMotor.getCurrentPosition();
//        int positionError = targetPosition - currentPosition;
//        // Apply P-control to resist external movement and hold position
//        double power = Kp * positionError;  // Proportional control
//        power = Math.max(Math.min(power, 0.5), -0.5);  // Limit power between -1 and 1
//        // Apply calculated power to the motor
//        sliderMotor.setPower(power);
//
//
//        // Display telemetry data
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Slider Position", currentPosition);
//        telemetry.addData("Target Position", targetPosition);
//        telemetry.addData("Position Error", positionError);
//        telemetry.addData("Motor Power", power);
//        telemetry.update();
//    }
//
//    private void debounceDelay() {
//        try {
//            sleep(300); // Debounce delay
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//    }
//
//    @Override
//    public void stop() {
//        // Stop all motors when the game is stopped
//        sliderMotor.setPower(0);
//    }
//}