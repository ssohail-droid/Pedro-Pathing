//package pedroPathing.SubSystem.Opmodes;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import java.util.HashMap;
//import java.util.List;
//import java.util.Map;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@TeleOp(name = "Basic AutoNav + Limelight", group = "Test")
//public class BasicAutoNavLimelight extends OpMode {
//
//    private Follower follower;
//    private Limelight3A limelight;
//    private boolean limelightConnected = false;
//
//    // Robot starting pose
//    private final Pose startPose = new Pose(0, 0, 0);
//
//    // Target point to auto navigate to
//    private final Pose autoNavTarget = new Pose(24, 24, 0);
//
//    private boolean hasNavigated = false;
//
//    // Limelight mount offset
//    public static double LIMELIGHT_X_OFFSET = 6.0;
//    public static double LIMELIGHT_Y_OFFSET = 0.0;
//    public static double LIMELIGHT_HEADING_OFFSET = 0.0;
//
//    // Known field positions of AprilTags
//    private final Map<Integer, TagFieldPose> tagFieldPoses = new HashMap<>();
//
//    private static class TagFieldPose {
//        double x, y, headingRad;
//
//        TagFieldPose(double x, double y, double headingDeg) {
//            this.x = x;
//            this.y = y;
//            this.headingRad = Math.toRadians(headingDeg);
//        }
//    }
//
//    @Override
//    public void init() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        try {
//            limelight = hardwareMap.get(Limelight3A.class, "limelight");
//            limelight.pipelineSwitch(0);
//            limelight.start();
//            limelightConnected = true;
//        } catch (Exception e) {
//            limelightConnected = false;
//        }
//
//        // Example: Add tag 4 at field position (60, 36), facing downfield
//        tagFieldPoses.put(4, new TagFieldPose(60, 36, 0));
//    }
//
//    @Override
//    public void start() {
//        follower.followPath(
//                follower.pathBuilder()
//                        .addPath(new com.pedropathing.pathgen.BezierLine(
//                                new com.pedropathing.pathgen.Point(startPose),
//                                new com.pedropathing.pathgen.Point(autoNavTarget)))
//                        .setLinearHeadingInterpolation(startPose.getHeading(), autoNavTarget.getHeading())
//                        .build()
//        );
//        hasNavigated = true;
//    }
//
//    @Override
//    public void loop() {
//        // ✅ Always correct pose from Limelight if AprilTag is visible
//        correctOdometryFromLimelight();
//
//        // If still navigating, update path
//        if (hasNavigated && !follower.hasFinishedFollowing()) {
//            follower.update();
//        } else {
//            // Joystick driving after autoNav is done
//            double drive = -gamepad1.left_stick_y;
//            double strafe = -gamepad1.left_stick_x;
//            double turn = -gamepad1.right_stick_x;
//
//            follower.setTeleOpMovementVectors(strafe, drive, turn, false);
//            follower.update();
//        }
//
//        // Basic telemetry
//        Pose pose = follower.getPose();
//        telemetry.addData("X", "%.1f", pose.getX());
//        telemetry.addData("Y", "%.1f", pose.getY());
//        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
//        telemetry.update();
//    }
//
//    private void correctOdometryFromLimelight() {
//        if (!limelightConnected) return;
//
//        LLResult result = limelight.getLatestResult();
//        if (result == null || !result.isValid()) return;
//
//        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//        if (tags == null || tags.isEmpty()) return;
//
//        LLResultTypes.FiducialResult tag = tags.get(0);
//        int tagID = tag.getID();
//        if (!tagFieldPoses.containsKey(tagID)) return;
//
//        Pose3D cameraPoseRelToTag = tag.getCameraPose();
//        if (cameraPoseRelToTag == null) return;
//
//        double tx = cameraPoseRelToTag.getTranslation().get(0);
//        double ty = cameraPoseRelToTag.getTranslation().get(1);
//        double theta = cameraPoseRelToTag.getRotation().get(2);
//
//        // Invert transform
//        double invX = -tx * Math.cos(theta) + -ty * Math.sin(theta);
//        double invY = -tx * Math.sin(theta) - -ty * Math.cos(theta);
//        double invTheta = -theta;
//
//        TagFieldPose tagPose = tagFieldPoses.get(tagID);
//
//        double robotX = tagPose.x + invX * Math.cos(tagPose.headingRad) - invY * Math.sin(tagPose.headingRad);
//        double robotY = tagPose.y + invX * Math.sin(tagPose.headingRad) + invY * Math.cos(tagPose.headingRad);
//        double robotHeading = normalizeAngle(tagPose.headingRad + invTheta);
//
//        // Apply camera offset
//        robotX += LIMELIGHT_X_OFFSET * Math.cos(robotHeading) - LIMELIGHT_Y_OFFSET * Math.sin(robotHeading);
//        robotY += LIMELIGHT_X_OFFSET * Math.sin(robotHeading) + LIMELIGHT_Y_OFFSET * Math.cos(robotHeading);
//        robotHeading = normalizeAngle(robotHeading + LIMELIGHT_HEADING_OFFSET);
//
//        follower.setPose(new Pose(robotX, robotY, robotHeading));
//    }
//
//    private double normalizeAngle(double angle) {
//        while (angle > Math.PI) angle -= 2 * Math.PI;
//        while (angle < -Math.PI) angle += 2 * Math.PI;
//        return angle;
//    }
//}
