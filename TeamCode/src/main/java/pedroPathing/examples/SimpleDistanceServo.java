package pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Z_Simple_Sensor_Servo - THE SIMPLEST CODE.
 *
 * It triggers a servo to close when the distance is under 5 cm.
 *
 * REQUIRED CONFIG NAMES:
 * 1. Distance Sensor: "dist_sensor"
 * 2. Servo: "my_servo"
 */
//@TeleOp(name = "Z_Simple_Sensor_Servo", group = "Simple")
public class SimpleDistanceServo extends OpMode {

    // Declare objects (short names for simplicity)
    DistanceSensor dist;
    Servo servo;

    // --- INIT ---
    @Override
    public void init() {
        // Find hardware (MUST match config names exactly!)
        dist = hardwareMap.get(DistanceSensor.class, "shootDe");
        servo = hardwareMap.get(Servo.class, "hold_servo");

        servo.setPosition(0.3); // Set servo to an 'open' position
        telemetry.addData("Status", "READY. Check names: dist_sensor, my_servo");
        telemetry.update();
    }

    // --- LOOP ---
    @Override
    public void loop() {
        // Read distance in CM. If hardware name is wrong, this line will crash!
        double currentDistanceCM = dist.getDistance(DistanceUnit.CM);

        if (currentDistanceCM < 5.0) {
            // Under 5 cm: Set servo to 'closed'
            servo.setPosition(0.3);
            telemetry.addData("ACTION", "CLOSING");
        } else {
            // 5 cm or more: Set servo to 'open'
            servo.setPosition(0.6);
            telemetry.addData("ACTION", "OPEN");
        }

        // Display distance
        telemetry.addData("Distance", "%.1f cm", currentDistanceCM);
        telemetry.update();
    }

    // --- STOP ---
    @Override
    public void stop() {
        // We do not need any code here for this simple OpMode.
    }
}