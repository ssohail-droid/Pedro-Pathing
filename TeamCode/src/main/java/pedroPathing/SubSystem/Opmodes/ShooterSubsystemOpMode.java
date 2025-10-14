//package pedroPathing.SubSystem.Opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import pedroPathing.SubSystem.ShooterSubsystem;
//
//@TeleOp(name="Shooter Subsystem OpMode", group="Subsystems")
//public class ShooterSubsystemOpMode extends LinearOpMode {
//    private ShooterSubsystem shooter;
//
//    @Override
//    public void runOpMode() {
//        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
//        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");
//        shooter = new ShooterSubsystem(shooter1, shooter2);
//
//        telemetry.addLine("Shooter ready.");
//        telemetry.addLine("A = Spin up shooter");
//        telemetry.addLine("B = Set idle");
//        telemetry.addLine("X = Stop shooter");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // A: Spin up shooter
//            if (gamepad1.a) {
//                shooter.spinUp();
//            }
//
//            // B: Set to idle
//            if (gamepad1.b) {
//                shooter.setIdle();
//            }
//
//            // X: Stop shooter completely
//            if (gamepad1.x) {
//                shooter.stop();
//            }
//
//            telemetry.addData("Shooter Spinning", shooter.isSpinning());
//            telemetry.update();
//        }
//    }
//}
