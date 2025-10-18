package pedroPathing.SubSystem.Opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.SubSystem.TransferSubsystem;

@Disabled
@TeleOp(name="Transfer (Feed) Subsystem OpMode", group="Subsystems")
public class TransferSubsystemOpMode extends LinearOpMode {
    private TransferSubsystem transfer;

    @Override
    public void runOpMode() {
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        transfer = new TransferSubsystem(feedMotor);

        telemetry.addLine("Transfer ready.");
        telemetry.addLine("A = Start feed");
        telemetry.addLine("B = Stop feed");
        telemetry.addLine("X = Reverse feed");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // A: Start transfer/feed
            if (gamepad1.a) {
                transfer.start();
            }

            // B: Stop transfer/feed
            if (gamepad1.b) {
                transfer.stop();
            }

            // X: Reverse transfer/feed
            if (gamepad1.x) {
                transfer.reverse();
            }

            telemetry.addData("Transfer Running", transfer.isRunning());
            telemetry.update();
        }
    }
}
