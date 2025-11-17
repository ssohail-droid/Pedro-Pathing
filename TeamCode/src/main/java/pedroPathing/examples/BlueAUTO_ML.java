//package pedroPathing.examples;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//import pedroPathing.SubSystem.ShooterSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//import java.io.BufferedWriter;
//import java.io.File;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.List;
//
//@Config
//@Autonomous(name = "BlueAUTO ML Enhanced v7", group = "ML Examples")
//public class BlueAUTO_ML extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//    private int pathState;
//
//    private IntakeSubsystem intake;
//    private ShooterSubsystem shooter;
//    private TransferSubsystem transfer;
//    private ServoSubsystem servos;
//
//    private VoltageSensor voltageSensor;
//    private double currentVoltage = 12.5;
//    private double adaptiveMaxPower = 0.4;
//    private MultipleTelemetry multiTelemetry;
//
//    private int shotStep = 0;
//    private int totalShots = 4;
//    private int currentShot = 0;
//    private boolean rpmDipped = false;
//    private long stepStartTime = 0;
//
//    private final int Heading = 180;
//    private final Pose onePos = new Pose(134.3, 34, Math.toRadians(Heading));
//    private final Pose twoPos = new Pose(118.5, 34, Math.toRadians(Heading));
//
//    private PathChain moveOne;
//    private PathChain moveTwo;
//
//    private List<MLDataPoint> trainingData = new ArrayList<>();
//    private long shotStartTime = 0;
//    private Pose shotStartPose;
//    private int successfulShots = 0;
//    private int timeoutShots = 0;
//
//    private static class MLDataPoint {
//        double voltage, targetRPM, actualRPM, shotDuration, xError, yError;
//        boolean rpmDipped, timeout;
//        long timestamp;
//
//        @Override
//        public String toString() {
//            return String.format("%.2f,%.1f,%.1f,%.0f,%b,%b,%.2f,%.2f,%d",
//                    voltage, targetRPM, actualRPM, shotDuration, rpmDipped, timeout, xError, yError, timestamp);
//        }
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        Constants.setConstants(FConstants.class, LConstants.class);
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(onePos);
//        buildPaths();
//
//        // ====== Updated Servo Initialization ======
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
//        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
//        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");
//        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
//
//        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        shooter = new ShooterSubsystem(hardwareMap, multiTelemetry);
//        intake = new IntakeSubsystem(intakeMotor);
//        transfer = new TransferSubsystem(feedMotor);
//        servos = new ServoSubsystem(pushServo);
//        servos.setHoldServo(holdServo); // Added per ServoSubsystem spec
//
//        // ==========================================
//
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        currentVoltage = voltageSensor.getVoltage();
//
//        multiTelemetry.addLine("=== ML ENHANCEMENTS ===");
//        multiTelemetry.addData("Current Voltage", "%.2f V", currentVoltage);
//        multiTelemetry.update();
//    }
//
//    public void autonomousPathUpdate() {
//        currentVoltage = voltageSensor.getVoltage();
//
//        switch (pathState) {
//            case 0:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(adaptiveMaxPower);
//                    follower.followPath(moveOne);
//                    setPathState(1);
//                }
//                break;
//
//            case 1:
//                shooter.update();
//
//                switch (shotStep) {
//                    case 0:
//                        servos.engagePush();
//                        rpmDipped = false;
//                        stepStartTime = System.currentTimeMillis();
//                        shotStartTime = System.currentTimeMillis();
//                        shotStartPose = follower.getPose();
//                        shotStep = 1;
//                        break;
//
//                    case 1:
//                        if (shooter.getRPM() >= ShooterSubsystem.targetRPM * 0.95) {
//                            servos.retractPush();
//                            stepStartTime = System.currentTimeMillis();
//                            shotStep = 2;
//                        }
//                        break;
//
//                    case 2:
//                        if (System.currentTimeMillis() - stepStartTime >= 250) {
//                            intake.start();
//                            transfer.start();
//                            stepStartTime = System.currentTimeMillis();
//                            shotStep = 3;
//                        }
//                        break;
//
//                    case 3:
//                        if (System.currentTimeMillis() - stepStartTime <= 5000) {
//                            if (shooter.getRPM() < ShooterSubsystem.targetRPM * 0.92) {
//                                rpmDipped = true;
//                                servos.engagePush();
//                                successfulShots++;
//                                stepStartTime = System.currentTimeMillis();
//                                shotStep = 5;
//                            }
//                        } else {
//                            timeoutShots++;
//                            stepStartTime = System.currentTimeMillis();
//                            shotStep = 4;
//                        }
//                        break;
//
//                    case 4:
//                        // ==== Updated fallback uses Hold servo ====
//                        servos.engageHold();
//                        if (System.currentTimeMillis() - stepStartTime >= 1000) {
//                            servos.retractHold();
//                            servos.engageHold();
//                            stepStartTime = System.currentTimeMillis();
//                            shotStep = 5;
//                        }
//                        break;
//
//                    case 5:
//                        intake.stop();
//                        transfer.stop();
//
//                        if (shooter.getRPM() >= ShooterSubsystem.targetRPM * 0.95) {
//                            currentShot++;
//                            if (currentShot < totalShots) {
//                                shotStep = 0;
//                            } else {
//                                setPathState(2);
//                            }
//                        }
//                        break;
//                }
//                break;
//
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(moveTwo);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    public void buildPaths() {
//        moveOne = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(onePos), new Point(twoPos)))
//                .setLinearHeadingInterpolation(onePos.getHeading(), twoPos.getHeading())
//                .build();
//
//        moveTwo = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(twoPos), new Point(onePos)))
//                .setLinearHeadingInterpolation(twoPos.getHeading(), onePos.getHeading())
//                .build();
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        shooter.update();
//        autonomousPathUpdate();
//
//        multiTelemetry.addData("Shot Step", shotStep);
//        multiTelemetry.addData("Current Shot", "%d / %d", currentShot, totalShots);
//        multiTelemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        intake.stop();
//        transfer.stop();
//        servos.emergencyStop();
//    }
//}
