package pedroPathing.examples;


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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "State machine with pathing", group = "Examples")
public class StateMachineWithPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int facing = 180;

    DcMotor linearMotor; // variables must be created here but initialized in loop()
    Servo wrist;
    Servo claw;
    private int armPickup;
    private int armClip;
    private int armClipDown;
    private double moterSpeed;

    private final Pose startPos = new Pose(7.117, 71.644, Math.toRadians(facing)); // point A
    private final Pose Submersible = new Pose(43, 69, Math.toRadians(facing)); // point B
    private final Pose Submersible2 = new Pose(48, 69, Math.toRadians(facing)); // point B

///

    private final Pose SubmersibleBack = new Pose(34, 69, Math.toRadians(facing));
    private final Pose clearBrace = new Pose(34, 15, Math.toRadians(facing));





    private PathChain goToSubmersible;
    private PathChain goToSubmersible2;
    private PathChain moveBackFromSubmersible;
    private PathChain moveClearBrace;

    public void buildPaths() {

        goToSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(Submersible)))
                .setLinearHeadingInterpolation(startPos.getHeading(), Submersible.getHeading())
                .build();

        goToSubmersible2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Submersible), new Point(Submersible2)))
                .setLinearHeadingInterpolation(Submersible.getHeading(), Submersible2.getHeading())
                .build();

        moveBackFromSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Submersible2), new Point(SubmersibleBack)))
                .setLinearHeadingInterpolation(Submersible2.getHeading(), SubmersibleBack.getHeading())
                .build();

        moveClearBrace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SubmersibleBack), new Point(clearBrace)))
                .setLinearHeadingInterpolation(SubmersibleBack.getHeading(), clearBrace.getHeading())
                .build();
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
                    follower.setMaxPower(0.4);

                    follower.followPath(goToSubmersible);
                    setPathState(1);
                }

                break;
            case 1:

                if(!follower.isBusy()) {

                    follower.setMaxPower(0.4);
                    follower.followPath(goToSubmersible2);
                    setPathState(2);


                }
                if (follower.getCurrentTValue() > 0.9){


                    linearMotor.setTargetPosition(armClipDown);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                    //claw.setPosition(0.15);
                    //wrist.setPosition(0.7);



                }
                break;


            case 2:

                if(!follower.isBusy()) {
                    follower.setMaxPower(0.4);

                    follower.followPath(moveBackFromSubmersible);

                    setPathState(3);
                }



                break;

            case 3:

                if(!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    follower.followPath(moveClearBrace);


                    if (follower.getCurrentTValue() > 0.0){


                        linearMotor.setTargetPosition(armPickup);
                        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearMotor.setPower(moterSpeed);
                        //claw.setPosition(0.15);
                        //wrist.setPosition(0.7);



                    }

                    setPathState(4);
                }


                break;

            case 4:

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
        armClip = -405;

        armClipDown =-370;

        moterSpeed = 0.67;

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
