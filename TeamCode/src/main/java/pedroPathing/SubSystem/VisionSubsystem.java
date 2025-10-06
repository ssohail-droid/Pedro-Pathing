package pedroPathing.SubSystem;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionSubsystem {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    private final int centerX = 320;
    private final int deadZone = 40;
    private final int[] validTagIDs = {20, 21, 22, 23};
    private Integer trackedTagID = null;

    public static class AlignmentData {
        public final double turnCorrection;
        public final double forwardPower;
        public final double strafePower;
        public final int tagID;
        public final double errorX;

        public AlignmentData(double turn, double forward, double strafe, int id, double error) {
            this.turnCorrection = turn;
            this.forwardPower = forward;
            this.strafePower = strafe;
            this.tagID = id;
            this.errorX = error;
        }
    }

    public VisionSubsystem(VisionPortal portal, AprilTagProcessor processor) {
        this.visionPortal = portal;
        this.aprilTag = processor;
    }

    public AlignmentData getAlignmentData(double targetDistanceForwardPower) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection target = null;

        for (AprilTagDetection detection : detections) {
            if (trackedTagID != null && detection.id == trackedTagID) {
                target = detection;
                break;
            } else if (trackedTagID == null && isValidTag(detection.id)) {
                trackedTagID = detection.id;
                target = detection;
                break;
            }
        }

        if (target != null) {
            double errorX = target.center.x - centerX;
            double kP = 0.0015;
            double turnCorrection = Math.abs(errorX) > deadZone ? errorX * kP : 0.0;

            // forwardPower is provided by caller (OpMode/command decides)
            double forward = -targetDistanceForwardPower;
            return new AlignmentData(turnCorrection, forward, 0.0, target.id, errorX);
        }

        trackedTagID = null;
        return null;
    }

    public void resetTracking() { trackedTagID = null; }

    private boolean isValidTag(int id) {
        for (int validID : validTagIDs) if (id == validID) return true;
        return false;
    }

    public void close() { visionPortal.close(); }
}
