package pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Differential Claw (Servo Mode)", group = "TeleOp")
public class DifferentialClawServoMode extends OpMode {

    private Servo leftServo;
    private Servo rightServo;



    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Reverse one of the servos so they move in sync for open/close
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            // Open the claw
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);
            telemetry.addData("wrist", "mid");
        } else if (gamepad1.b) {
            // Close the claw
            leftServo.setPosition(0.8);
            rightServo.setPosition(0.8);
            telemetry.addData("Claw", "Closing");
        }else if (gamepad1.x) {
            // Close the claw
            leftServo.setPosition(0.2);
            rightServo.setPosition(0.2);
            telemetry.addData("Claw", "Closing");
        }

        telemetry.update();
    }
}
