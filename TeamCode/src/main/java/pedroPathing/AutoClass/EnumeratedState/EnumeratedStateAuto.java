package pedroPathing.AutoClass.EnumeratedState;

import static pedroPathing.AutoClass.EnumeratedState.State.STATE_START;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


//@Autonomous(name = "EnumeratedStateAuto(V1.0)", group = "Examples")
public class EnumeratedStateAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private State state = STATE_START;
    private boolean pathStarted = false;
    private final int Heading = 180;
    private final int SamplePushWall = 20;// Flag to ensure followPath is called once per state

    DcMotor linearMotor; // variables must be created here but initialized in loop()
    Servo wrist;
    Servo claw;

    private int armPickup;
    private int armClip;
    private int armClipDown;


    private double motorSpeed;


    // Pose variables - starting and target positions
    private final Pose startPos = new Pose(9, 72, Math.toRadians(180));
    private final Pose submersiblePos = new Pose(33.5, 72, Math.toRadians(180));
    private final Pose SubmersibleScorePos = new Pose(33.7, 72, Math.toRadians(Heading));
    private final Pose SubmersibleBackPos = new Pose(24, 72, Math.toRadians(Heading));

    private PathChain moveToSubmersible;
    private PathChain moveToSubmersible2;
    private PathChain moveBackFromSubmersible;


    public void buildPaths() {
        moveToSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(submersiblePos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), submersiblePos.getHeading())
                .build();

        moveToSubmersible2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePos), new Point(SubmersibleScorePos)))
                .setLinearHeadingInterpolation(submersiblePos.getHeading(), SubmersibleScorePos.getHeading())
                .build();

        moveBackFromSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SubmersibleScorePos), new Point(SubmersibleBackPos)))
                .setLinearHeadingInterpolation(SubmersibleScorePos.getHeading(), SubmersibleBackPos.getHeading())
                .build();




    }

    public void autonomousPathUpdate() {
        switch (state) {
            case STATE_START:
                // Wait until follower is idle, then transition to move to submersible
                if (!follower.isBusy()) {
                    setPathState(State.STATE_MOVE_TO_SUBMERSIBLE);
                    pathStarted = false; // Reset flag for next state
                }
                break;

            case STATE_MOVE_TO_SUBMERSIBLE:
                // Start following path once
                if (!pathStarted) {
                    follower.setMaxPower(0.5);
                    follower.followPath(moveToSubmersible);
                    pathStarted = true;
                }

                if (follower.getCurrentTValue() > 0.0){
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);
                    claw.setPosition(1.0);

                }

//                6/

                // When path is complete, transition to next state
                if (pathStarted && !follower.isBusy()) {
                    setPathState(State.STATE_MOVE_TO_AND_SCORE);
                    pathStarted = false;
                }
                break;

            case STATE_MOVE_TO_AND_SCORE:
                // Start following return path once
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(moveToSubmersible2);


                    pathStarted = true;
                }

                // When path is complete, transition to HOME state
                if (pathStarted && !follower.isBusy()) {
                    setPathState(State.STATE_MOVE_BACK_FROM_SUBMERSIBLE);
                    pathStarted = false;
                }
                break;

            case STATE_MOVE_BACK_FROM_SUBMERSIBLE:
                // Start following return path once
                if (!pathStarted) {
                    follower.setMaxPower(1);
                    follower.followPath(moveBackFromSubmersible);


                    pathStarted = true;
                }

                // When path is complete, transition to HOME state
                if (pathStarted && !follower.isBusy()) {
                    setPathState(State.STATE_HOME);
                    pathStarted = false;
                }
                break;


            case STATE_HOME:
                // Final state — can add more logic here if needed
                break;

            default:
                // Safety fallback: do nothing or log unexpected state
                telemetry.addData("Error", "Unknown state: " + state);
                telemetry.update();
                break;
        }
    }

    public void setPathState(State pState) {
        state = pState;
        pathTimer.resetTimer();
        telemetry.addData("State changed to", state);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        buildPaths();

        linearMotor = hardwareMap.get(DcMotor.class, "lift");
        linearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        claw = hardwareMap.get(Servo.class, "leftServo");

        claw.setPosition(1.0);


    }

    @Override
    public void loop() {



        armPickup = -1053;
        armClip = -415;
        armClipDown =-500;


        motorSpeed = 0.8;

        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(State.STATE_START);
        pathStarted = false;
    }

    @Override
    public void stop() {}
}
