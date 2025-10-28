//package pedroPathing.SubSystem.Opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//
//@TeleOp(name = "Red-Green LED Test", group = "Test")
//public class RedGreenLEDTest extends LinearOpMode {
//
//    private DigitalChannel ledRed;
//    private DigitalChannel ledGreen;
//
//    @Override
//    public void runOpMode() {
//        // Get LEDs from hardware map
//        ledRed = hardwareMap.get(DigitalChannel.class, "led1");
//        ledGreen = hardwareMap.get(DigitalChannel.class, "led2");
//
//        // Set mode to OUTPUT
//        ledRed.setMode(DigitalChannel.Mode.OUTPUT);
//        ledGreen.setMode(DigitalChannel.Mode.OUTPUT);
//
//        // Turn both off initially (HIGH = off for active-low devices)
//        ledRed.setState(true);
//        ledGreen.setState(true);
//
//        telemetry.addData("Status", "Ready to control Red/Green LED");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                // RED ON, GREEN OFF
//                ledRed.setState(false);   // active LOW
//                ledGreen.setState(true);
//            } else if (gamepad1.b) {
//                // GREEN ON, RED OFF
//                ledRed.setState(true);
//                ledGreen.setState(false);
//            } else if (gamepad1.x) {
//                // BOTH OFF
//                ledRed.setState(true);
//                ledGreen.setState(true);
//            }
//
//            telemetry.addData("Red LED", ledRed.getState() ? "OFF" : "ON");
//            telemetry.addData("Green LED", ledGreen.getState() ? "OFF" : "ON");
//            telemetry.update();
//        }
//    }
//}
