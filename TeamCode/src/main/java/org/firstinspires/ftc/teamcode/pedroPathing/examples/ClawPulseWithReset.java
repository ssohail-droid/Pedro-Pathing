//package pedroPathing.examples;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//@Disabled
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Claw Pulse with Reset", group = "TeleOp")
//public class ClawPulseWithReset extends LinearOpMode {
//
//    CRServo leftServo;
//    CRServo rightServo;
//
//    boolean previousA = false;
//    boolean previousB = false;
//    boolean previousRBumper = false;
//
//    @Override
//    public void runOpMode() {
//        leftServo = hardwareMap.get(CRServo.class, "leftServo");
//        rightServo = hardwareMap.get(CRServo.class, "rightServo");
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            boolean currentA = gamepad1.a;
//            boolean currentB = gamepad1.b;
//            boolean currentRBumper = gamepad1.right_bumper;
//
//            // A: Pulse forward
//            if (currentA && !previousA) {
//                leftServo.setPower(1.0);
//                rightServo.setPower(-1.0);
//                sleep(200);
//                leftServo.setPower(0);
//                rightServo.setPower(0);
//            }
//
//            // B: Pulse backward
//            if (currentB && !previousB) {
//                leftServo.setPower(-1.0);
//                rightServo.setPower(1.0);
//                sleep(200);
//                leftServo.setPower(0);
//                rightServo.setPower(0);
//            }
//
//            // Right bumper: Reset to original position
//            if (currentRBumper && !previousRBumper) {
//                // Customize this direction as needed!
//                leftServo.setPower(1.0);
//                rightServo.setPower(1.0);
//                sleep(700);  // Maybe a bit longer for full reset
//                leftServo.setPower(0);
//                rightServo.setPower(0);
//            }
//
//            // Update previous button states
//            previousA = currentA;
//            previousB = currentB;
//            previousRBumper = currentRBumper;
//
//            telemetry.addLine("A: Forward | B: Reverse | Right Bumper: Reset");
//            telemetry.update();
//        }
//    }
//}
