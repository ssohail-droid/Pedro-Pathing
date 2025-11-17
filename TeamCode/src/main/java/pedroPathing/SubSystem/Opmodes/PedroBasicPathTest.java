//package pedroPathing.SubSystem.Opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@Autonomous(name = "PedroHeadingMoveTest", group = "Test")
//public class PedroBasicPathTest extends OpMode {
//
//    private Follower follower;
//
//    private PathChain moveToTarget;
//    private PathChain moveBack;
//
//    private int pathState = 0;
//
//    // === Start and End Poses ===
//    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//    private final Pose targetPose = new Pose(24, 0, Math.toRadians(90));  // Move forward and rotate to 90°
//
//    @Override
//    public void init() {
//        // Initialize Pedro constants
//        Constants.setConstants(FConstants.class, LConstants.class);
//
//        // Create follower
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Build paths
//        buildPaths();
//        telemetry.addLine("Initialized. Ready to start.");
//        telemetry.update();
//    }
//
//    public void buildPaths() {
//        // Move forward 24 inches and rotate from 0 to 90 degrees
//        moveToTarget = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(targetPose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
//                .build();
//
//        // Move back to start and rotate from 90° to 0°
//        moveBack = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(targetPose), new Point(startPose)))
//                .setLinearHeadingInterpolation(targetPose.getHeading(), startPose.getHeading())
//                .build();
//    }
//
//    @Override
//    public void start() {
//        pathState = 0;
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        switch (pathState) {
//            case 0:
//                // Start move to target
//                if (!follower.isBusy()) {
//                    follower.followPath(moveToTarget);
//                    pathState = 1;
//                }
//                break;
//
//            case 1:
//                // Wait for move to finish
//                if (!follower.isBusy()) {
//                    pathState = 2;
//                }
//                break;
//
//            case 2:
//                // Move back to start
//                follower.followPath(moveBack);
//                pathState = 3;
//                break;
//
//            case 3:
//                // Wait for return to complete
//                if (!follower.isBusy()) {
//                    pathState = -1; // Done
//                }
//                break;
//        }
//
//        // Always show pose and status
//        Pose pose = follower.getPose();
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("X", "%.2f", pose.getX());
//        telemetry.addData("Y", "%.2f", pose.getY());
//        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        // Optional: Stop motors or log final pose
//    }
//}
