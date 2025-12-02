package pedroPathing.AutoClass.Dec6;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor; // Added for alignment
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // Added for distance reading

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "BlueAUTO DEC", group = "Examples")
public class BlueAUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState; // Path sequence: 0->1 (Shoot) -> 4 (Align) -> 2 -> 3 (End)

    private final int Heading = 0;
    private final Pose onePos = new Pose(16.5, 35, Math.toRadians(90));
    private final Pose twoPos = new Pose(20, 31, Math.toRadians(73.4));

    private PathChain moveOne;
    private PathChain moveTwo;

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;

    // --- Servo Fields (now defined directly in OpMode) ---
    private Servo holdServo;  // hold_servo (gate)
    private Servo pushServo2; // push_servo (pusher)

    // --- Servo Positions (Placeholder values - adjust based on your hardware) ---
    private static final double HOLD_GATE_CLOSED = 0.6; // Position to close the gate
    private static final double HOLD_GATE_OPEN = 0.2;   // Position to open the gate
    private static final double PUSHER_EXTEND = 0.9;    // Position to extend the pusher
    private static final double PUSHER_RETRACT = 0.1;   // Position to retract the pusher

    // --- Distance Sensors for Alignment ---
    private DistanceSensor frontSensor;
    // private DistanceSensor rightSensor; // Removed right sensor
    private long alignStartTime; // To track alignment timeout




    // --- Shooting State Machine Variables ---
    private int shotStep = 0;
    private int totalShots = 4;
    private int currentShot = 0;
    private boolean rpmDipped = false;
    private double targetRPM = ShooterSubsystem.targetRPM;
    private long stepStartTime = 0;

    // ------------------------------------------------------------

    public void buildPaths() {
        moveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(onePos), new Point(twoPos)))
                .setLinearHeadingInterpolation(onePos.getHeading(), twoPos.getHeading())
                .build();

        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twoPos), new Point(onePos)))
                .setLinearHeadingInterpolation(twoPos.getHeading(), onePos.getHeading())
                .build();
    }

    /**
     * Executes the sensor-based auto-alignment routine using only the front sensor.
     * Corrects only forward/backward movement (Y-axis).
     */


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Initial move to shooting position
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveOne);

                    setPathState(1);
                }
                break;

            case 1:
                // End autonomous
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

        // Specific setup for alignment state
        if (pState == 4) {
            follower.breakFollowing(); // Stop any current path following
            follower.startTeleopDrive(); // Switch follower to manual/TeleOp drive mode
            alignStartTime = System.currentTimeMillis(); // Start timeout timer
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(onePos);
        buildPaths();



        // Initialize subsystems
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");

        // --- Servo Initialization (Directly in OpMode) ---
        holdServo = hardwareMap.get(Servo.class, "hold_servo");   // hold gate
        pushServo2 = hardwareMap.get(Servo.class, "push_servo");  // pusher

        // Set initial positions (Gate closed, Pusher retracted)
        holdServo.setPosition(HOLD_GATE_CLOSED);
        pushServo2.setPosition(PUSHER_RETRACT);

        shooter = new ShooterSubsystem(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        intake = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update(); // Always keep shooter updated

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shot Step", shotStep);
        telemetry.addData("Current Shot", currentShot);
        telemetry.addData("Shooter RPM", "%.1f", shooter.getRPM());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        // --- Alignment Telemetry ---
        if (pathState == 4 || pathState == 1) {
            telemetry.addLine("=== Alignment Sensors ===");
            telemetry.addData("Aligning", pathState == 4);
            telemetry.addData("Front (in)", "%.2f", frontSensor.getDistance(DistanceUnit.INCH));
            // telemetry.addData("Right (in)", "%.2f", rightSensor.getDistance(DistanceUnit.INCH)); // Removed
            if (pathState == 4) {
                telemetry.addData("Time Elapsed (ms)", System.currentTimeMillis() - alignStartTime);
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
        transfer.stop();
    }
}