package pedroPathing.SubSystem.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import pedroPathing.SubSystem.VisionSubsystem;

@TeleOp(name="All Mechanisms (No Drive)", group="Subsystems")
public class AllMechanismsNoDriveOpMode extends LinearOpMode {

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;
    private VisionSubsystem vision;

    private static final double TARGET_DISTANCE = 0.25;

    @Override
    public void runOpMode() {
        // Initialize hardware
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Initialize subsystems
        intake = new IntakeSubsystem(intakeMotor);
        shooter = new ShooterSubsystem(shooter1, shooter2);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(holdServo, pushServo);
        vision = new VisionSubsystem(portal, aprilTag);

        telemetry.addLine("All mechanisms ready. No drive.");
        telemetry.addLine("GP1: Intake/Transfer control");
        telemetry.addLine("GP2: Shooter/Servo control");
        telemetry.addLine("Dpad Up: Vision alignment data");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ===== GAMEPAD 1: Intake & Transfer =====
            // Left trigger: Start intake
            if (gamepad1.left_trigger > 0.5) {
                intake.start();
            }
            // Left bumper: Stop intake
            if (gamepad1.left_bumper) {
                intake.stop();
            }

            // Right trigger: Start transfer
            if (gamepad1.right_trigger > 0.5) {
                transfer.start();
            }
            // Right bumper: Stop transfer
            if (gamepad1.right_bumper) {
                transfer.stop();
            }

            // ===== GAMEPAD 2: Shooter & Servos =====
            // A: Spin up shooter
            if (gamepad2.a) {
                shooter.spinUp();
            }
            // B: Set shooter to idle
            if (gamepad2.b) {
                shooter.setIdle();
            }
            // X: Stop shooter
            if (gamepad2.x) {
                shooter.stop();
            }

            // Y: Engage hold servo
            if (gamepad2.y) {
                servos.engageHold();
            }
            // Dpad Down: Release hold servo
            if (gamepad2.dpad_down) {
                servos.releaseHold();
            }
            // Dpad Left: Engage push servo
            if (gamepad2.dpad_left) {
                servos.engagePush();
            }
            // Dpad Right: Retract push servo
            if (gamepad2.dpad_right) {
                servos.retractPush();
            }

            // ===== Vision (Dpad Up on GP1) =====
            if (gamepad1.dpad_up) {
                VisionSubsystem.AlignmentData data = vision.getAlignmentData(TARGET_DISTANCE);
                if (data != null) {
                    telemetry.addData("Vision Tag", data.tagID);
                    telemetry.addData("ErrorX", "%.2f", data.errorX);
                    telemetry.addData("Turn", "%.3f", data.turnCorrection);
                } else {
                    telemetry.addLine("No tag detected");
                }
            }

            // Telemetry
            telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
            telemetry.addData("Transfer", transfer.isRunning() ? "Running" : "Stopped");
            telemetry.addData("Shooter", shooter.isSpinning() ? "Spinning" : "Idle/Stopped");
            telemetry.update();
        }

        vision.close();
    }
}
