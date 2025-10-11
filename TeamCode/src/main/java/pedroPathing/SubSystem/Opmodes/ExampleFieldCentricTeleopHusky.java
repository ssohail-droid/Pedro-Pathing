package pedroPathing.SubSystem.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * Field-Centric TeleOp with HuskyLens Auto-Pan
 *
 * Robot rotation is automatically controlled to track Block ID 1
 */
@TeleOp(name = "Example Field-Centric Teleop with HuskyLens", group = "Examples")
public class ExampleFieldCentricTeleopHusky extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    private HuskyLens huskylens;

    // HuskyLens auto-pan parameters
    private final int centerX = 160;       // HuskyLens frame center
    private final int deadZone = 5;        // pixels dead zone
    private final double turnGain = 0.003; // proportional gain
    private final double detectionHoldTime = 2.0; // seconds tolerance

    private ElapsedTime detectionTimer = new ElapsedTime();
    private boolean lastDetection = false;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        huskylens = hardwareMap.get(HuskyLens.class, "hs");
        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addData("Init", huskylens.knock() ? "✅ HuskyLens connected" : "❌ Problem communicating with HuskyLens");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        follower.startTeleopDrive();
        detectionTimer.reset();
    }

    @Override
    public void loop() {

        // --- HuskyLens Auto-Pan Calculation ---
        HuskyLens.Block[] blocks = huskylens.blocks();
        double panTurn = 0;
        boolean hasBlock1 = false;

        if (blocks.length > 0) {
            for (HuskyLens.Block block : blocks) {
                if (block.id == 1) {
                    hasBlock1 = true;

                    double panError = block.x - centerX;
                    if (Math.abs(panError) < deadZone) panError = 0;

                    panTurn = panError * turnGain;

                    telemetry.addData("Block X", block.x);
                    telemetry.addData("Turn Power", "%.3f", panTurn);
                    break; // track first Block ID 1 only
                }
            }
        }

        // --- Tolerance to prevent twitching ---
        if (hasBlock1) {
            detectionTimer.reset();
            lastDetection = true;
        }

        double timeSinceLastDetection = detectionTimer.seconds();
        if (!lastDetection || timeSinceLastDetection > detectionHoldTime) {
            panTurn = 0;
            lastDetection = false;
        }

        // --- Pedro Pathing TeleOp Drive ---
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y, // forward/back
                -gamepad1.left_stick_x, // strafe
                panTurn,                // rotation from HuskyLens
                false                   // robot-centric mode
        );

        follower.update();

        // --- Telemetry ---
        telemetry.addData("Detected Blocks", blocks.length);
        telemetry.addData("Last Seen (s)", "%.2f", timeSinceLastDetection);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {}
}
