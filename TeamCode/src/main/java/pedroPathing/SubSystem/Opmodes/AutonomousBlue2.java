//package pedroPathing.SubSystem.Opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.SubSystem.ShooterSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//@Autonomous(name = "Auto - Shoot Balls (RPM Dip + Fallback Push)", group = "Subsystems")
//public class AutonomousBlue2 extends LinearOpMode {
//
//    private IntakeSubsystem intake;
//    private ShooterSubsystem shooter;
//    private TransferSubsystem transfer;
//    private ServoSubsystem servos;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
//        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
//        Servo pushServo = hardwareMap.get(Servo.class, "hold_servo");       // hold gate
//        Servo pushServo2 = hardwareMap.get(Servo.class, "push_servo");    // pusher
//
//        shooter = new ShooterSubsystem(hardwareMap,
//                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
//        intake = new IntakeSubsystem(intakeMotor);
//        transfer = new TransferSubsystem(feedMotor);
//        servos = new ServoSubsystem(pushServo);
//        servos.setPushServo2(pushServo2); // ✅ attach second servo
//
//        telemetry.addLine("Auto Shoot Ready — press PLAY to start");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        shooter.update();
//        sleep(100); // spin-up buffer
//
//        double targetRPM = ShooterSubsystem.targetRPM;
//        int totalShots = 3;
//
//        for (int i = 0; i < totalShots && opModeIsActive(); i++) {
//            // 1️⃣ Close hold gate to start
//            servos.engagePush(); // Closed = no ball enters yet
//
//            // 2️⃣ Wait for shooter to reach target RPM
//            while (opModeIsActive() && shooter.getRPM() < targetRPM * 0.95) {
//                shooter.update();
//                telemetry.addData("Shooter RPM", "%.1f", shooter.getRPM());
//                telemetry.addData("Waiting for spin-up...", "%d/%d", i + 1, totalShots);
//                telemetry.update();
//            }
//
//            // 3️⃣ Open hold gate to allow next ball in
//            servos.retractPush(); // Open = let ball fall in
//            telemetry.addLine("Hold servo opened");
//            telemetry.update();
//
//            // 4️⃣ Wait a short delay, then start intake + transfer
//            sleep(250); // Optional short buffer before feeding
//            intake.start();
//            transfer.start();
//            telemetry.addLine("Intake + transfer started");
//            telemetry.update();
//
//            // 5️⃣ Watch for RPM dip — wait up to 3 seconds
//            long startTime = System.currentTimeMillis();
//            boolean rpmDipped = false;
//
//            while (opModeIsActive() && System.currentTimeMillis() - startTime < 3000) {
//                shooter.update();
//                double currentRPM = shooter.getRPM();
//
//                if (currentRPM < targetRPM * 0.92) {
//                    rpmDipped = true;
//
//                    // 6️⃣ RPM dip detected → close hold gate
//                    servos.engagePush(); // Close = stop next ball
//                    telemetry.addLine("Shot detected — hold gate closed");
//                    telemetry.update();
//                    break;
//                }
//
//                telemetry.addData("Shooter RPM", "%.1f", currentRPM);
//                telemetry.addLine("Waiting for RPM dip...");
//                telemetry.update();
//            }
//
//            // 7️⃣ If RPM did not dip → use push servo to fire
//            if (!rpmDipped && opModeIsActive()) {
//                telemetry.addLine("No RPM dip — using push servo");
//                telemetry.update();
//
//                servos.engagePush2();   // Fully extend push servo
//                sleep(3000);             // ✅ long enough for full push
//                servos.retractPush2();  // Retract after full extension
//
//                // Still close hold gate after push
//                servos.engagePush();
//            }
//
//            // 8️⃣ Stop intake and transfer
//            intake.stop();
//            transfer.stop();
//
//            // 9️⃣ Wait for shooter to return to target RPM
//            while (opModeIsActive() && shooter.getRPM() < targetRPM * 0.95) {
//                shooter.update();
//                telemetry.addLine("Waiting for RPM recovery...");
//                telemetry.addData("Shooter RPM", "%.1f", shooter.getRPM());
//                telemetry.update();
//            }
//
//            // 🔟 Wait before next shot
//            telemetry.addLine("Waiting before next shot...");
//            telemetry.update();
//            sleep(3000); // optional delay between shots
//        }
//    }
//}