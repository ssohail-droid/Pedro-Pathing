//package pedroPathing.SubSystem.Opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Point;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.SubSystem.ShooterSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//
//@Autonomous(name = "AutoBlue - Pedro Backward Half", group = "Subsystems")
//public class AutonomousBlue3 extends LinearOpMode {
//
//    private Follower follower;
//
//    private IntakeSubsystem intake;
//    private ShooterSubsystem shooter;
//    private TransferSubsystem transfer;
//    private ServoSubsystem servos;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // === Set constants for Pedro ===
//        com.pedropathing.util.Constants.setConstants(FConstants.class, LConstants.class);
//
//        // === Hardware setup ===
//        follower = new Follower(hardwareMap);
//
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
//        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
//        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
//        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");
//
//        shooter = new ShooterSubsystem(hardwareMap,
//                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
//        intake = new IntakeSubsystem(intakeMotor);
//        transfer = new TransferSubsystem(feedMotor);
//        servos = new ServoSubsystem(holdServo);
//        servos.setPushServo2(pushServo);
//
//        ShooterSubsystem.targetRPM = 2800;
//
//        // === Define start and end poses ===
//        double headingDeg = -225;
//        double headingRad = Math.toRadians(headingDeg);
//
//        // Start pose: near the blue goal wall
//        Pose startPose = new Pose(50, 180, headingRad);
//
//        // Distance: HALF of 3000 ticks → ~35 inches
//        double ticksPerInch = 537.6 / (Math.PI * 4); // ~42.78
//        double distanceInInches = 1500 / ticksPerInch; // ≈ 35 inches
//
//        // Move BACKWARDS from heading → use -cos/sin
//        double dx = -Math.cos(headingRad) * distanceInInches;
//        double dy = -Math.sin(headingRad) * distanceInInches;
//
//        // Calculate end position (backward along heading)
//        Pose endPose = new Pose(startPose.getX() + dx, startPose.getY() + dy, headingRad);
//
//        // Set starting pose
//        follower.setStartingPose(startPose);
//
//        telemetry.addLine("Auto ready: Pedro will drive backward (half) and shoot");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // === Step 1: Pedro drive BACKWARDS (35 inches along heading 135°) ===
//        follower.followPath(follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(startPose),
//                        new Point(endPose)))
//                .setLinearHeadingInterpolation(headingRad, headingRad) // maintain heading
//                .setReversed(true) // DRIVE BACKWARDS
//                .build());
//
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            shooter.update(); // spin up while moving
//            telemetry.addLine("Pedro driving backward...");
//            telemetry.addData("Target RPM", ShooterSubsystem.targetRPM);
//            telemetry.addData("Current RPM", "%.1f", shooter.getRPM());
//            telemetry.update();
//        }
//
//        // === Step 2: Shooting sequence ===
//        shooter.update();
//        sleep(100);
//        double targetRPM = ShooterSubsystem.targetRPM;
//        int totalShots = 3;
//        long startAuto = System.currentTimeMillis();
//
//        while (opModeIsActive() && shooter.getRPM() < targetRPM * 0.95) {
//            shooter.update();
//            telemetry.addLine("Spinning up shooter...");
//            telemetry.addData("Target RPM", targetRPM);
//            telemetry.addData("Current RPM", "%.1f", shooter.getRPM());
//            telemetry.update();
//        }
//
//        for (int i = 0; i < totalShots && opModeIsActive(); i++) {
//            servos.engagePush();
//            while (opModeIsActive() && shooter.getRPM() < targetRPM * 0.95) {
//                shooter.update();
//                telemetry.addData("Shooter RPM", "%.1f", shooter.getRPM());
//                telemetry.addLine("Waiting for spin-up...");
//                telemetry.update();
//            }
//
//            servos.retractPush();
//            sleep(250);
//            intake.start();
//            transfer.start();
//
//            long startTime = System.currentTimeMillis();
//            boolean rpmDipped = false;
//
//            while (opModeIsActive() && System.currentTimeMillis() - startTime < 3000) {
//                shooter.update();
//                double currentRPM = shooter.getRPM();
//
//                if (currentRPM < targetRPM * 0.92) {
//                    rpmDipped = true;
//                    servos.engagePush();
//                    telemetry.addLine("Shot detected — hold closed");
//                    telemetry.addData("Target RPM", targetRPM);
//                    telemetry.addData("Current RPM", "%.1f", currentRPM);
//                    telemetry.update();
//                    break;
//                }
//
//                telemetry.addData("Shooter RPM", "%.1f", currentRPM);
//                telemetry.addData("Target RPM", targetRPM);
//                telemetry.update();
//            }
//
//            if (!rpmDipped && opModeIsActive()) {
//                servos.engagePush2();
//                sleep(3000);
//                servos.retractPush2();
//                servos.engagePush();
//            }
//
//            intake.stop();
//            transfer.stop();
//
//            while (opModeIsActive() && shooter.getRPM() < targetRPM * 0.95) {
//                shooter.update();
//                telemetry.addLine("Waiting RPM recovery...");
//                telemetry.addData("Target RPM", targetRPM);
//                telemetry.addData("Current RPM", "%.1f", shooter.getRPM());
//                telemetry.update();
//            }
//
//            sleep(1000);
//
//            if (System.currentTimeMillis() - startAuto > 25000) break;
//        }
//
//        shooter.stop();
//        telemetry.addLine("Auto complete — finished shooting");
//        telemetry.update();
//    }
//}
