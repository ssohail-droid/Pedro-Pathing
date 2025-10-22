package pedroPathing.SubSystem.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Autonomous(name = "Auto - Shoot Sequence (RPM Dip Control)", group = "Subsystems")
public class AutonomousBlue2 extends LinearOpMode {

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");

        shooter = new ShooterSubsystem(hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        intake = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(pushServo);

        telemetry.addLine("Auto Shoot Ready — press PLAY to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Start systems
        intake.start();
        transfer.start();
        servos.engagePush();

        boolean rpmDipDetected = false;
        boolean feedActive = true;

        while (opModeIsActive()) {
            shooter.update();
            double currentRPM = shooter.getRPM();
            double targetRPM = ShooterSubsystem.targetRPM;

            // Detect dip — shooter RPM drops below 90% of target
            if (currentRPM < targetRPM * 0.90 && feedActive) {
                rpmDipDetected = true;
                feedActive = false;
                intake.stop();
                transfer.stop();
                servos.retractPush();
            }

            // Detect recovery — shooter back above 95% of target
            if (currentRPM >= targetRPM * 0.95 && !feedActive) {
                rpmDipDetected = false;
                feedActive = true;
                intake.start();
                transfer.start();
                servos.engagePush();
            }

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", currentRPM);
            telemetry.addData("RPM Dip", rpmDipDetected ? "detected" : "none");
            telemetry.addData("Feed Active", feedActive ? "yes" : "no");
            telemetry.update();
        }

        // Safety stop
        shooter.stop();
        intake.stop();
        transfer.stop();
        servos.retractPush();
    }
}
