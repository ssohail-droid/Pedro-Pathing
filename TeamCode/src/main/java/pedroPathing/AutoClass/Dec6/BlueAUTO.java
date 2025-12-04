package pedroPathing.AutoClass.Dec6;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

//@Autonomous(name = "BlueAUTO DEC", group = "Examples")
public class BlueAUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose onePos = new Pose(16.5, 35, Math.toRadians(90));
    private final Pose twoPos = new Pose(20, 31, Math.toRadians(73.4));
    private final Pose threePos = new Pose(66, 50, Math.toRadians(270));
    private final Pose fourPos = new Pose(66, 27, Math.toRadians(270));
    private final Pose fivePos = new Pose(90, 50, Math.toRadians(270));
    private final Pose sixPos = new Pose(90, 27, Math.toRadians(270));
    private final Pose sevenPos = new Pose(43, 18.5, Math.toRadians(90));

    private PathChain moveOne, moveTwo, moveThree, moveFour;
    private PathChain moveFive, moveSix, moveSeven, moveEight, moveNine;

    private DcMotorEx shooterMotor, transferMotor, intakeMotor;
    private Servo gateServo, pusherServo;
    private DistanceSensor beamSensor;

    private ShooterManager shooterManager;

    public void buildPaths() {
        moveOne = follower.pathBuilder().addPath(new BezierLine(new Point(onePos), new Point(twoPos)))
                .setLinearHeadingInterpolation(onePos.getHeading(), twoPos.getHeading()).build();

        moveTwo = follower.pathBuilder().addPath(new BezierLine(new Point(twoPos), new Point(threePos)))
                .setLinearHeadingInterpolation(twoPos.getHeading(), threePos.getHeading()).build();

        moveThree = follower.pathBuilder().addPath(new BezierLine(new Point(threePos), new Point(fourPos)))
                .setLinearHeadingInterpolation(threePos.getHeading(), fourPos.getHeading()).build();

        moveFour = follower.pathBuilder().addPath(new BezierLine(new Point(fourPos), new Point(fivePos)))
                .setLinearHeadingInterpolation(fourPos.getHeading(), fivePos.getHeading()).build();

        moveFive = follower.pathBuilder().addPath(new BezierLine(new Point(fivePos), new Point(twoPos)))
                .setLinearHeadingInterpolation(fivePos.getHeading(), twoPos.getHeading()).build();

        moveSix = follower.pathBuilder().addPath(new BezierLine(new Point(twoPos), new Point(fivePos)))
                .setLinearHeadingInterpolation(twoPos.getHeading(), fivePos.getHeading()).build();

        moveSeven = follower.pathBuilder().addPath(new BezierLine(new Point(fivePos), new Point(sixPos)))
                .setLinearHeadingInterpolation(fivePos.getHeading(), sixPos.getHeading()).build();

        moveEight = follower.pathBuilder().addPath(new BezierLine(new Point(sixPos), new Point(twoPos)))
                .setLinearHeadingInterpolation(sixPos.getHeading(), twoPos.getHeading()).build();

        moveNine = follower.pathBuilder().addPath(new BezierLine(new Point(twoPos), new Point(sevenPos)))
                .setLinearHeadingInterpolation(twoPos.getHeading(), sevenPos.getHeading()).build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(moveOne);
                    setPathState(1);
                }
                break;

            case 1:
                shooterManager.startShooting();
                setPathState(2);
                break;

            case 2:
                shooterManager.update();
                if (shooterManager.getState().equals("DONE")) {
                    intakeMotor.setPower(1.0);
                    follower.followPath(moveTwo);
                    setPathState(-3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(moveThree);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(moveFour);
                    setPathState(5);
                }
                break;

            case 5:
                shooterManager.startShooting();
                setPathState(6);
                break;

            case 6:
                shooterManager.update();
                if (shooterManager.getState().equals("DONE")) {
                    intakeMotor.setPower(1.0);
                    follower.followPath(moveFive);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(moveSix);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(moveSeven);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.followPath(moveEight);
                    setPathState(10);
                }
                break;

            case 10:
                shooterManager.startShooting();
                setPathState(11);
                break;

            case 11:
                shooterManager.update();
                if (shooterManager.getState().equals("DONE")) {
                    follower.followPath(moveNine);
                    setPathState(12);
                }
                break;

            case 12:
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

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(12.0, 0.05, 8.0, 14.6);

        transferMotor = hardwareMap.get(DcMotorEx.class, "feed_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");

        gateServo = hardwareMap.get(Servo.class, "hold_servo");
        pusherServo = hardwareMap.get(Servo.class, "push_servo");
        beamSensor = hardwareMap.get(DistanceSensor.class, "shootDe");

        shooterManager = new ShooterManager();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter RPM", shooterManager.getShooterRPM());
        telemetry.addData("Shooter State", shooterManager.getState());
        telemetry.update();
    }

    @Override
    public void stop() {}

    private class ShooterManager {
        private String shooterState = "IDLE";
        private ElapsedTime timer = new ElapsedTime();

        private final double BEAM_THRESHOLD_CM = 7.0;
        private final double GATE_OPEN = 0.3;
        private final double GATE_CLOSED = 0.6;
        private final double PUSHER_PUSH = 0.9;
        private final double PUSHER_RETRACT = 0.3;
        private final double TICKS_PER_REV = 28.0;
        private final double targetRPM = 2500;
        private final double targetTPS = (targetRPM * TICKS_PER_REV) / 60.0;

        private final long RECOVERY_TIMEOUT_MS = 1000;
        private final long PUSHER_DELAY_MS = 250;

        public void startShooting() {
            shooterMotor.setVelocity(targetTPS);
            gateServo.setPosition(GATE_OPEN);
            pusherServo.setPosition(PUSHER_RETRACT);
            transferMotor.setPower(1.0);
            intakeMotor.setPower(1.0);
            shooterState = "IDLE";
            timer.reset();
        }

        public void update() {
            double beamDist = beamSensor.getDistance(DistanceUnit.CM);
            boolean beamBroken = beamDist < BEAM_THRESHOLD_CM;

            switch (shooterState) {
                case "IDLE":
                    if (beamBroken) {
                        transferMotor.setPower(0);
                        gateServo.setPosition(GATE_CLOSED);
                        timer.reset();
                        shooterState = "BEAM_BROKEN";
                    }
                    break;

                case "BEAM_BROKEN":
                    if (isShooterReady() || timer.milliseconds() > RECOVERY_TIMEOUT_MS) {
                        gateServo.setPosition(GATE_OPEN);
                        transferMotor.setPower(1.0);
                        timer.reset();
                        shooterState = "FEEDING_NEXT";
                    }
                    break;

                case "FEEDING_NEXT":
                    if (beamBroken) {
                        transferMotor.setPower(0);
                        gateServo.setPosition(GATE_CLOSED);
                        timer.reset();
                        shooterState = "RECOVERING";
                    } else if (timer.milliseconds() > 1000) {
                        transferMotor.setPower(0);
                        shooterState = "FINAL_PUSH";
                        timer.reset();
                    }
                    break;

                case "RECOVERING":
                    if (isShooterReady() || timer.milliseconds() > RECOVERY_TIMEOUT_MS) {
                        gateServo.setPosition(GATE_OPEN);
                        transferMotor.setPower(1.0);
                        timer.reset();
                        shooterState = "FINAL_PUSH";
                    }
                    break;

                case "FINAL_PUSH":
                    pusherServo.setPosition(PUSHER_PUSH);
                    if (timer.milliseconds() > PUSHER_DELAY_MS) {
                        pusherServo.setPosition(PUSHER_RETRACT);
                        transferMotor.setPower(0);
                        intakeMotor.setPower(0);
                        shooterState = "DONE";
                    }
                    break;

                case "DONE":
                    shooterMotor.setVelocity(0);
                    transferMotor.setPower(0);
                    intakeMotor.setPower(0);
                    break;
            }
        }

        public boolean isShooterReady() {
            double currentTPS = shooterMotor.getVelocity();
            double currentRPM = (currentTPS * 60.0) / TICKS_PER_REV;
            return Math.abs(currentRPM - targetRPM) < 100;
        }

        public double getShooterRPM() {
            return (shooterMotor.getVelocity() * 60.0) / TICKS_PER_REV;
        }

        public String getState() {
            return shooterState;
        }
    }
}