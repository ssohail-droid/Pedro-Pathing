package pedroPathing.SubSystem.Opmodes;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;

@Disabled
@TeleOp(name = "HuskyLens Auto Intake (Tolerance)", group = "Sensor")
public class HuskyLensAutoIntake extends LinearOpMode {

    private HuskyLens huskylens;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private ElapsedTime detectionTimer = new ElapsedTime();
    private boolean lastDetection = false;

    // You can adjust this to fine-tune the delay
    private final double detectionHoldTime = 2.0; // seconds of tolerance

    @Override
    public void runOpMode() {
        huskylens = hardwareMap.get(HuskyLens.class, "hs");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake = new IntakeSubsystem(intakeMotor);

        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        transfer = new TransferSubsystem(feedMotor);

        telemetry.addData("Init", huskylens.knock() ? "✅ HuskyLens connected" : "❌ Problem communicating with HuskyLens");
        telemetry.update();

        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        waitForStart();
        detectionTimer.reset();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskylens.blocks();
            boolean hasBlock1 = false;

            for (HuskyLens.Block block : blocks) {
                if (block.id == 1) {
                    hasBlock1 = true;
                    break;
                }
            }

            // 🕒 Manage tolerance window
            if (hasBlock1) {
                detectionTimer.reset();
                lastDetection = true;
            }

            double timeSinceLastDetection = detectionTimer.seconds();

            if (lastDetection && timeSinceLastDetection < detectionHoldTime) {
                // Recently detected Block 1
                transfer.start();
                intake.start();
                telemetry.addLine("✅ Block 1 active (within tolerance) → Intake FORWARD");
            } else {
                // Not seen for > 2 seconds
               // transfer.reverse();
                intake.reverse();
                telemetry.addLine("❌ Block 1 lost for > 2 s → Intake REVERSE");
                lastDetection = false;
            }

            telemetry.addData("Detected Blocks", blocks.length);
            telemetry.addData("Last seen (s)", "%.2f", timeSinceLastDetection);
            telemetry.update();
        }
    }
}
