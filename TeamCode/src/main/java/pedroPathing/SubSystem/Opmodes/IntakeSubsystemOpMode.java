package pedroPathing.SubSystem.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.SubSystem.IntakeSubsystem;

@TeleOp(name="Intake Subsystem OpMode", group="Subsystems")
public class IntakeSubsystemOpMode extends LinearOpMode {
    private IntakeSubsystem intake;

    @Override
    public void runOpMode() {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake = new IntakeSubsystem(intakeMotor);

        telemetry.addLine("Intake ready.");
        telemetry.addLine("A = Start intake");
        telemetry.addLine("B = Stop intake");
        telemetry.addLine("X = Reverse intake");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // A: Start intake
            if (gamepad1.a) {
                intake.start();
            }

            // B: Stop intake
            if (gamepad1.b) {
                intake.stop();
            }

            // X: Reverse intake
            if (gamepad1.x) {
                intake.reverse();
            }

            telemetry.addData("Intake Running", intake.isRunning());
            telemetry.update();
        }
    }
}
