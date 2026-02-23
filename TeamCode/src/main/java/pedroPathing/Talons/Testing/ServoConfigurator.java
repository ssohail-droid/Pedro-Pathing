package pedroPathing.Talons.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoConfigurator extends LinearOpMode {

    private Servo sorter;
    double servoPos;

    @Override
    public void runOpMode() throws InterruptedException {
        sorter = hardwareMap.get(Servo.class , "spin");

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.dpad_up){
                servoPos += .05;
            }
            else if (gamepad1.dpad_down){
                servoPos -=.05;
            }
            servoPos = servoPos < 0 ? 0 : servoPos > 1 ? 1 : servoPos;
            sorter.setPosition(servoPos);

            telemetry.addData("servo pos" , servoPos);
            telemetry.update();



        }

    }

}
