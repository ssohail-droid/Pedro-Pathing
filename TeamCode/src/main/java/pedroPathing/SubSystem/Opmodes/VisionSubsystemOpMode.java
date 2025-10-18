package pedroPathing.SubSystem.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import pedroPathing.SubSystem.VisionSubsystem;

@Disabled
@TeleOp(name="Vision Subsystem OpMode", group="Subsystems")
public class VisionSubsystemOpMode extends LinearOpMode {
    private VisionSubsystem vision;
    private static final double TARGET_DISTANCE = 0.25;

    @Override
    public void runOpMode() {
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        vision = new VisionSubsystem(portal, aprilTag);

        telemetry.addLine("Vision ready.");
        telemetry.addLine("Hold A = Get alignment data");
        telemetry.addLine("B = Reset tracking");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // A: Get alignment data
            if (gamepad1.a) {
                VisionSubsystem.AlignmentData data = vision.getAlignmentData(TARGET_DISTANCE);
                if (data != null) {
                    telemetry.addData("Tag ID", data.tagID);
                    telemetry.addData("Error X", "%.2f", data.errorX);
                    telemetry.addData("Turn Correction", "%.3f", data.turnCorrection);
                    telemetry.addData("Forward Power", "%.3f", data.forwardPower);
                    telemetry.addData("Strafe Power", "%.3f", data.strafePower);
                } else {
                    telemetry.addLine("No valid AprilTag detected");
                }
            } else {
                telemetry.addLine("Hold A to get alignment data");
            }

            // B: Reset tracking
            if (gamepad1.b) {
                vision.resetTracking();
                telemetry.addLine("Tracking reset");
            }

            telemetry.update();
        }

        vision.close();
    }
}
