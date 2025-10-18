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

@Autonomous(name = "BlueAUTO", group = "Examples")
public class BlueAUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final int Heading = 0;

    private final Pose onePos = new Pose(134.3, 34, 0);
    private final Pose TwoPos = new Pose(90, 34, Math.toRadians(Heading));



    private PathChain moveOne;
    private PathChain moveTwo;
    private PathChain moveThree;
    private PathChain moveFour;



    public void buildPaths() {

        moveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(onePos), new Point(TwoPos)))
                .setLinearHeadingInterpolation(onePos.getHeading(), TwoPos.getHeading())
                .build();

        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(TwoPos), new Point(onePos)))
                .setLinearHeadingInterpolation(TwoPos.getHeading(), onePos.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            /*end of unit test one*/
            case 0:


                if(!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    follower.followPath(moveOne);
                    setPathState(1);
                }

                break;

            case 1:


                if(!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    follower.followPath(moveTwo);
                    setPathState(2);
                }

                break;

            case 2:

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
        follower.setStartingPose(onePos);
        buildPaths();





    }

    @Override
    public void loop() {



        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop() {



    }

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


