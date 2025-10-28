//package pedroPathing.SubSystem.Opmodes;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//@Config
//@TeleOp(name = "Shooter Tuner Dashboard", group = "Testing")
//public class ShooterTuner extends LinearOpMode {
//
//    // Adjustable via Dashboard
//    public static double targetRPM = 3000;
//
//    public static double kP = 22.0;
//    public static double kI = 0.0;
//    public static double kD = 8.0;
//    public static double kF = 11.5;
//
//    private final double TICKS_PER_REV = 28.0;
//
//    private DcMotorEx shooter;
//
//    @Override
//    public void runOpMode() {
//
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooter.setDirection(DcMotorSimple.Direction.FORWARD); // Change to REVERSE if needed
//
//        // Set initial PIDF coefficients
//        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
//
//        // Telemetry to both Driver Station and Dashboard
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetry.addLine("Shooter Tuner Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // Update PIDF live (optional — only needed if changing during run)
//            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
//
//            // Set target velocity (in ticks/sec)
//            double targetVelocity = (targetRPM * TICKS_PER_REV) / 60.0;
//            shooter.setVelocity(targetVelocity);
//
//            // Read actual velocity
//            double actualVelocity = shooter.getVelocity(); // ticks/sec
//            double actualRPM = (actualVelocity * 60.0) / TICKS_PER_REV;
//
//            // Telemetry
//            telemetry.addData("Target RPM", targetRPM);
//            telemetry.addData("Actual RPM", actualRPM);
//            telemetry.addData("Target Velocity", targetVelocity);
//            telemetry.addData("Actual Velocity", actualVelocity);
//            telemetry.addData("RPM Error", targetRPM - actualRPM);
//            telemetry.update();
//
//            // Dashboard packet (optional, for graphing)
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("Target RPM", targetRPM);
//            packet.put("Actual RPM", actualRPM);
//            packet.put("Error", targetRPM - actualRPM);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//            sleep(50);
//        }
//
//        shooter.setPower(0); // Stop motor when OpMode ends
//    }
//}
