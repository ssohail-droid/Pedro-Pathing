package pedroPathing.Jan10.TELEOP.TESTOP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

//@TeleOp(name = "Limelight → Shoot Pattern Select", group = "Test")
public class LimelightPatternSelect extends LinearOpMode {

    private Limelight3A limelight;

    /** Pipeline index set in Limelight UI (AprilTag 36h11) */
    private static final int APRILTAG_PIPELINE = 0;

    /** This is the value you will later pass into your auto */
    private int selectedPattern = 0;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Limelight ready");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            telemetry.addData("LL Name", status.getName());
            telemetry.addData(
                    "LL Status",
                    "Temp: " + JavaUtil.formatNumber(status.getTemp(), 1) + "C, " +
                            "CPU: " + JavaUtil.formatNumber(status.getCpu(), 1) + "%, " +
                            "FPS: " + Math.round(status.getFps())
            );
            telemetry.addData("Pipeline", status.getPipelineIndex());

            if (result == null) {
                telemetry.addLine("No Limelight result");
                telemetry.update();
                continue;
            }

// General result info
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Latency (ms)", result.getCaptureLatency() + result.getTargetingLatency());
            telemetry.addData("BotPose", botpose != null ? botpose.toString() : "null");

// ===== APRILTAG PROCESSING =====
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            telemetry.addData("Tag Count", fiducials.size());

            if (!fiducials.isEmpty()) {

// Use the FIRST detected tag (simple + deterministic)
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                int tagId = tag.getFiducialId();

                telemetry.addData(
                        "Detected Tag",
                        "ID=" + tagId +
                                " Family=" + tag.getFamily() +
                                " tx=" + JavaUtil.formatNumber(tag.getTargetXDegrees(), 2) +
                                " ty=" + JavaUtil.formatNumber(tag.getTargetYDegrees(), 2)
                );

// ===== PATTERN SELECTION =====
// CHANGE THESE IDS TO MATCH YOUR FIELD TAGS
                if (tagId == 1) {
                    selectedPattern = 0;
                } else if (tagId == 2) {
                    selectedPattern = 1;
                } else if (tagId == 3) {
                    selectedPattern = 2;
                }
            }

            telemetry.addData("SELECTED_PATTERN", selectedPattern);
            telemetry.update();
        }

        limelight.stop();
    }
}