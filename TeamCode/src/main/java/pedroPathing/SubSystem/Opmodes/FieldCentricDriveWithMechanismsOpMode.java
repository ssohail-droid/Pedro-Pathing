package pedroPathing.SubSystem.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import pedroPathing.SubSystem.VisionSubsystem;

/**
 * Field-centric teleop with all subsystems integrated.
 *
 * Controls:
 * GAMEPAD 1 (Driver):
 * - Left Stick Y: Forward/Backward
 * - Left Stick X: Strafe Left/Right
 * - Right Stick X: Rotate
 * - Left Trigger: Start Intake
 * - Left Bumper: Stop Intake
 * - Right Trigger: Start Transfer
 * - Right Bumper: Stop Transfer
 * - Dpad Up: Get Vision Alignment Data
 *
 * GAMEPAD 2 (Operator):
 * - A: Spin Up Shooter
 * - B: Set Shooter to Idle
 * - X: Stop Shooter
 * - Y: Engage Hold Servo
 * - Dpad Down: Release Hold Servo
 * - Dpad Left: Engage Push Servo
 * - Dpad Right: Retract Push Servo
 */
@TeleOp(name = "Field-Centric Drive with Mechanisms", group = "Subsystems")
public class FieldCentricDriveWithMechanismsOpMode extends OpMode {

    // PedroPathing follower for field-centric drive
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // Subsystems
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;
    private VisionSubsystem vision;

    private static final double TARGET_DISTANCE = 0.25;

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void init() {
        // Initialize PedroPathing constants and follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize mechanism hardware
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        // Initialize vision
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Initialize subsystems
        intake = new IntakeSubsystem(intakeMotor);
        shooter = new ShooterSubsystem(shooter1, shooter2);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(holdServo, pushServo);
        vision = new VisionSubsystem(visionPortal, aprilTag);

        telemetry.addLine("Field-Centric Drive with Mechanisms Initialized");
        telemetry.addLine("GP1: Drive + Intake/Transfer + Vision");
        telemetry.addLine("GP2: Shooter + Servos");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // ===== FIELD-CENTRIC DRIVE (PedroPathing) =====
        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: false (field-centric)
        */
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        // ===== GAMEPAD 1: INTAKE & TRANSFER =====
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

        // ===== GAMEPAD 1: VISION =====
        // Dpad Up: Get vision alignment data
        if (gamepad1.dpad_up) {
            VisionSubsystem.AlignmentData data = vision.getAlignmentData(TARGET_DISTANCE);
            if (data != null) {
                telemetry.addLine("=== VISION ALIGNMENT ===");
                telemetry.addData("Tag ID", data.tagID);
                telemetry.addData("Error X", "%.2f", data.errorX);
                telemetry.addData("Turn Correction", "%.3f", data.turnCorrection);
                telemetry.addData("Forward Power", "%.3f", data.forwardPower);
            } else {
                telemetry.addLine("No AprilTag detected");
            }
        }

        // ===== GAMEPAD 2: SHOOTER =====
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

        // ===== GAMEPAD 2: SERVOS =====
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

        // ===== TELEMETRY =====
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("=== MECHANISMS ===");
        telemetry.addData("Intake", intake.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Transfer", transfer.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Shooter", shooter.isSpinning() ? "Spinning" : "Idle/Stopped");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up vision resources
        if (vision != null) {
            vision.close();
        }
    }
}
