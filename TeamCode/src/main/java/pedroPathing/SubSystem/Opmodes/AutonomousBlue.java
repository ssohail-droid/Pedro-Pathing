package pedroPathing.SubSystem.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Autonomous(name = "AutoShootSequence", group = "Subsystems")
public class AutonomousBlue extends LinearOpMode {

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;
    private ElapsedTime timer;

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
        timer = new ElapsedTime();

        telemetry.addLine("Auto Shoot Ready — press PLAY to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ======= STEP 1: Start shooter, intake, and feed =======
        shooter.update(); // start shooter (PIDF control)
        intake.start();
        transfer.start();
        telemetry.addLine("Shooter + Intake + Feed started");
        telemetry.update();

        // ======= STEP 2: Wait 3 seconds =======
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 3.0) {
            shooter.update(); // maintain RPM during wait
            telemetry.addData("Time", "%.1f", timer.seconds());
            telemetry.update();
        }

        // ======= STEP 3: Move servo =======
        servos.engagePush(); // set to your "fire" position
        telemetry.addLine("Servo activated");
        telemetry.update();
        sleep(1000); // wait 1 second for servo to move

        // ======= STEP 4: Stop everything =======
        shooter.stop();
        intake.stop();
        transfer.stop();
        servos.retractPush(); // reset servo

        telemetry.addLine("Sequence complete");
        telemetry.update();
        sleep(2000);
    }
}
