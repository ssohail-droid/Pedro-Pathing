package pedroPathing.examples;

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
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.SubSystem.IntakeSubsystem;
import pedroPathing.SubSystem.ServoSubsystem;
import pedroPathing.SubSystem.ShooterSubsystem;
import pedroPathing.SubSystem.TransferSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "BlueAUTOv7", group = "Examples")
public class BlueAUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final int Heading = 180;
    private final Pose onePos = new Pose(134.3, 34, Math.toRadians(Heading));
    private final Pose twoPos = new Pose(118.5, 34, Math.toRadians(Heading));

    private PathChain moveOne;
    private PathChain moveTwo;

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ServoSubsystem servos;

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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Initial move to shooting position
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    follower.followPath(moveOne);
                    setPathState(1);
                }
                break;

            case 1:
                // --- Shooting Logic from AutonomousBlue2 ---
                shooter.update(); // Always update shooter

                switch (shotStep) {
                    case 0:
                        // Step 0: Start spin-up
                        servos.engagePush(); // Close hold gate
                        rpmDipped = false;
                        stepStartTime = System.currentTimeMillis();
                        shotStep = 1;
                        break;

                    case 1:
                        // Step 1: Wait for spin-up
                        if (shooter.getRPM() >= targetRPM * 0.95) {
                            servos.retractPush(); // Open gate
                            stepStartTime = System.currentTimeMillis();
                            shotStep = 2;
                        }
                        break;

                    case 2:
                        // Step 2: Short delay, then start intake + transfer
                        if (System.currentTimeMillis() - stepStartTime >= 250) {
                            intake.start();
                            transfer.start();
                            stepStartTime = System.currentTimeMillis();
                            shotStep = 3;
                        }
                        break;

                    case 3:
                        // Step 3: Monitor RPM dip for 3 seconds
                        if (System.currentTimeMillis() - stepStartTime <= 5000) {
                            if (shooter.getRPM() < targetRPM * 0.92) {
                                rpmDipped = true;
                                servos.engagePush(); // Close hold gate
                                stepStartTime = System.currentTimeMillis();
                                shotStep = 5; // skip fallback
                            }
                        } else {
                            // Timeout → fallback push
                            stepStartTime = System.currentTimeMillis();
                            shotStep = 4;
                        }
                        break;

                    case 4:
                        // Step 4: Fallback push servo
                        servos.engagePush2(); // extend
                        if (System.currentTimeMillis() - stepStartTime >= 1000) {
                            servos.retractPush2(); // retract
                            servos.engagePush2();   // close gate
                            stepStartTime = System.currentTimeMillis();
                            shotStep = 5;
                        }
                        break;

                    case 5:
                        // Step 5: Stop intake + transfer
                        intake.stop();
                        transfer.stop();

                        // Wait for RPM recovery before next shot
                        if (shooter.getRPM() >= targetRPM * 0.95) {
                            currentShot++;
                            if (currentShot < totalShots) {
                                shotStep = 0; // repeat for next ball
                            } else {
                                // All shots done
                                setPathState(2);
                            }
                        }
                        break;
                }
                break;

            case 2:
                // Done shooting → go to next move
                if (!follower.isBusy()) {
                    follower.followPath(moveTwo);
                    setPathState(3);
                }
                break;

            case 3:
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
        Servo pushServo = hardwareMap.get(Servo.class, "hold_servo");   // hold gate
        Servo pushServo2 = hardwareMap.get(Servo.class, "push_servo");  // pusher

        shooter = new ShooterSubsystem(hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        intake = new IntakeSubsystem(intakeMotor);
        transfer = new TransferSubsystem(feedMotor);
        servos = new ServoSubsystem(pushServo);
        servos.setPushServo2(pushServo2);
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
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
        transfer.stop();
    }
}
