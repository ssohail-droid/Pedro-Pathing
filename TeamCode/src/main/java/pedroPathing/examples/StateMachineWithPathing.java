package pedroPathing.examples;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "State machine with pathing", group = "Examples")
public class StateMachineWithPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final int facing = 180;
    private final int SamplePushWall = 16;

    DcMotor linearMotor; // variables must be created here but initialized in loop()
    Servo wrist;
    Servo claw;
    private int armPickup;
    private int armClip;
    private int armClipDown;
    private double moterSpeed;
    private double pathSpeed;



    private final Pose startPos = new Pose(7.117, 71.644, Math.toRadians(facing));
    private final Pose SubmersiblePos = new Pose(43, 69, Math.toRadians(facing));
    private final Pose Submersible2Pos = new Pose(48, 69, Math.toRadians(facing));



    private final Pose SubmersibleBackPos = new Pose(34, 69, Math.toRadians(facing));
    private final Pose clearBracePos = new Pose(34, 15, Math.toRadians(facing));
    private final Pose behindSamplePos = new Pose(85, 15, Math.toRadians(facing));
    private final Pose behindSampleToPushPos = new Pose(85, 1, Math.toRadians(facing));


    private final Pose pushColourSampleOnePos = new Pose(SamplePushWall, 1, Math.toRadians(facing));
    private final Pose returnToSampleForPushPos = new Pose(85, -18, Math.toRadians(facing));
    private final Pose returnToSampleForPushControlPoint = new Pose(88, 25, Math.toRadians(facing));




    private PathChain moveToSubmersible;
    private PathChain moveToSubmersible2;
    private PathChain moveBackFromSubmersible;
    private PathChain moveClearBrace;
    private PathChain moveToBehindSample;
    private PathChain moveBehindSampleToPush;
    private PathChain movePushColourSampleOne;
    private Path movetoPushColourSampleTwo;

    public void buildPaths() {

        /*End of unit test 1.*/

        moveToSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(SubmersiblePos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), SubmersiblePos.getHeading())
                .build();

        moveToSubmersible2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SubmersiblePos), new Point(Submersible2Pos)))
                .setLinearHeadingInterpolation(SubmersiblePos.getHeading(), Submersible2Pos.getHeading())
                .build();

        moveBackFromSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Submersible2Pos), new Point(SubmersibleBackPos)))
                .setLinearHeadingInterpolation(Submersible2Pos.getHeading(), SubmersibleBackPos.getHeading())
                .build();


        /*^^^^^^All above unit test 1.^^^^^^*/

        /*End of unit test 2.*/

        moveClearBrace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SubmersibleBackPos), new Point(clearBracePos)))
                .setLinearHeadingInterpolation(SubmersibleBackPos.getHeading(), clearBracePos.getHeading())
                .build();

        moveToBehindSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clearBracePos), new Point(behindSamplePos)))
                .setLinearHeadingInterpolation(clearBracePos.getHeading(), behindSamplePos.getHeading())
                .build();


        moveBehindSampleToPush = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSamplePos), new Point(behindSampleToPushPos)))
                .setLinearHeadingInterpolation(behindSamplePos.getHeading(), behindSampleToPushPos.getHeading())
                .build();

        /*All above is unit test 2.*/

        /*End of unit test 3.*/

        movePushColourSampleOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSampleToPushPos), new Point(pushColourSampleOnePos)))
                .setLinearHeadingInterpolation(behindSampleToPushPos.getHeading(), pushColourSampleOnePos.getHeading())
                .build();

        movetoPushColourSampleTwo = new Path(new BezierCurve(new Point(pushColourSampleOnePos), /* Control Point */ new Point(returnToSampleForPushControlPoint), new Point(returnToSampleForPushPos)));
        movetoPushColourSampleTwo.setLinearHeadingInterpolation(pushColourSampleOnePos.getHeading(), returnToSampleForPushPos.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:

                if (follower.getCurrentTValue() > 0.0){
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                    claw.setPosition(0.15);
                    wrist.setPosition(0.7);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(pathSpeed);
                    follower.followPath(moveToSubmersible);
                    setPathState(1);
                }

                break;
            case 1:

                if(!follower.isBusy()) {

                    follower.setMaxPower(pathSpeed);
                    follower.followPath(moveToSubmersible2);

                    setPathState(2);

                }
                if (follower.getCurrentTValue() > 0.9){
                    linearMotor.setTargetPosition(armClipDown);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                    //claw.setPosition(0.0);
                    //wrist.setPosition(0.7);
                }

                break;


            case 2:

                if(!follower.isBusy()) {
                    follower.setMaxPower(pathSpeed);
                    follower.followPath(moveBackFromSubmersible);

                    setPathState(3);
                }

                break;

            case 3:

                if(!follower.isBusy()) {
                    follower.setMaxPower(pathSpeed);
                    follower.followPath(moveClearBrace);

                    if (follower.getCurrentTValue() > 0.0){
                        linearMotor.setTargetPosition(armPickup);
                        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearMotor.setPower(moterSpeed);
                    }
                    setPathState(4);
                }


                break;

            case 4:

                if(!follower.isBusy()) {

                    follower.setMaxPower(pathSpeed);
                    follower.followPath(moveToBehindSample);

                    setPathState(5);
                }
                break;

            case 5:

                if(!follower.isBusy()) {
                    follower.setMaxPower(pathSpeed);
                    follower.followPath(moveBehindSampleToPush);
                    setPathState(6);
                }
                break;

            case 6:

                if(!follower.isBusy()) {

                    follower.setMaxPower(pathSpeed);
                    follower.followPath(movePushColourSampleOne);
                    setPathState(7);
                }
                break;

            case 7:

                if(!follower.isBusy()) {
                    follower.setMaxPower(pathSpeed);
                    follower.followPath(movetoPushColourSampleTwo);
                    setPathState(8);
                }
                break;

            case 8:

                if(!follower.isBusy()) {

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
        follower.setStartingPose(startPos);
        buildPaths();

    }

    @Override
    public void loop() {

        linearMotor = hardwareMap.get(DcMotor.class, "lift");
        wrist = hardwareMap.get(Servo.class, "wrist"); // NAME WRIST SERVO CONFIGURATION TO wrist
        claw = hardwareMap.get(Servo.class, "claw"); // NAME CLAW SERVO CONFIGURATION TO claw


        armPickup = -1030;
        armClip = -410;
        armClipDown =-370;
        moterSpeed = 0.67;
        pathSpeed = 0.4;

        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
