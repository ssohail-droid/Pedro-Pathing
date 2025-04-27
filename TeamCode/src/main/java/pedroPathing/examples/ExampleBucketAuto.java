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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.AutoClass.Lift;
import pedroPathing.AutoClass.MotorTicController;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "DO NOT USE!!!!", group = "Examples")
public class ExampleBucketAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    DcMotor linearMotor; // variables must be created here but initialized in loop()
    Servo wrist;
    Servo claw;
    private int armPickup;
    private int armClip;

    private double moterSpeed = 0.67;

    // Ensure this matches your config

    // Initialize LiftController with appropriate limits



    // Set positions later





    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private int facing = 180;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.117, 71.644, Math.toRadians(facing));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(40, 69, Math.toRadians(facing));

    //temp
    private final Pose scorePoseOffset = new Pose(20, 69, Math.toRadians(facing));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(40, 12, Math.toRadians(facing));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(85, 15, Math.toRadians(facing));
// go to colour sample

//next go to push
    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(85, 1, Math.toRadians(facing));

    // behind the colour wall

    // push to wall

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose pickup4Pose = new Pose(14, -3, Math.toRadians(facing));

    // going back to colour sample

    private final Pose pickup5Pose = new Pose(85, -18, Math.toRadians(facing));

    private final Pose pickup5ControlPose = new Pose(64, 38, Math.toRadians(facing));

    // now puhsing to wall
    private final Pose wallPushPos = new Pose(14, -15, Math.toRadians(facing));

    private final Pose backOfColourSample = new Pose(85, -20, Math.toRadians(facing));
    private final Pose pickup6ControlPose = new Pose(71.64415156507414, 19, Math.toRadians(facing));

    private final Pose pushScore = new Pose(20, -15, Math.toRadians(facing));

    private final Pose scorePoseAfter = new Pose(29, 69, Math.toRadians(facing));



    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park,wallPush2;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3,wallPush , wallPush3, put2, put3, put4,put5,put6,home;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        // liftController = new LiftController(liftMotor, 0, 1000, 50);//Sohail did this

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line.*/
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line.*/
        grabPickup1 = follower.pathBuilder()
                //.addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .addPath(new BezierCurve(new Point(scorePose), new Point(scorePoseOffset), new Point(pickup1Pose)))

                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line.*/
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        //go to colour sample

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line.*/
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        //behins

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line.*/
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(pickup4Pose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup4Pose.getHeading())
                .build();


        park = new Path(new BezierCurve(new Point(pickup4Pose), /* Control Point */ new Point(pickup5ControlPose), new Point(pickup5Pose)));
        // the control point is the middle point in the BezierCurve parameters. the bezier line from the first point to last point gets pulled towards the control point to create a curve
        park.setLinearHeadingInterpolation(pickup4Pose.getHeading(), pickup5Pose.getHeading());

        wallPush = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup5Pose), new Point(wallPushPos)))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), wallPushPos.getHeading())
                .build();

        wallPush2 = new Path(new BezierCurve(new Point(wallPushPos), /* Control Point */ new Point(pickup6ControlPose), new Point(backOfColourSample)));
        wallPush2.setLinearHeadingInterpolation(wallPushPos.getHeading(), backOfColourSample.getHeading());

        wallPush3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(backOfColourSample), new Point(pushScore)))
                .setLinearHeadingInterpolation(backOfColourSample.getHeading(), pushScore.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line.*/

        put2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushScore), new Point(scorePose)))
                .setLinearHeadingInterpolation(pushScore.getHeading(), scorePose.getHeading())
                .build();

        put3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pushScore)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pushScore.getHeading())
                .build();

        put4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushScore), new Point(scorePose)))
                .setLinearHeadingInterpolation(pushScore.getHeading(), scorePose.getHeading())
                .build();

        put5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pushScore)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pushScore.getHeading())
                .build();

        put6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushScore), new Point(scorePose)))
                .setLinearHeadingInterpolation(pushScore.getHeading(), scorePose.getHeading())
                .build();

        home = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(startPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), startPose.getHeading())
                .build();



        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line.*
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point*/
        // park = new Path(new BezierCurve(new Point(pickup4Pose), /* Control Point */ new Point(pickup5ControlPose), new Point(pickup5Pose)));
        // park.setLinearHeadingInterpolation(pickup4Pose.getHeading(), pickup5Pose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {

        //double progress = follower.getFollowedPathProgress(1);







        switch (pathState) {
            case 0:



                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1: //drive forward to submersible

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */


                //DcMotor linearMotor = hardwareMap.get(DcMotor.class, "lift");





                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);

                }
                if (follower.getCurrentTValue() > 0.0){
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);



                }








                break;


            case 2: //drives right from submersible
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */




                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);//true
                    setPathState(3);





                }

                //DcMotor linearMotor2 = hardwareMap.get(DcMotor.class, "lift");

                if (follower.getCurrentTValue() > 0.7){
                    linearMotor.setTargetPosition(armPickup);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);


                }

                break;
            case 3: // drives slightly forward to be behind the samples
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4: //drives to be behind the samples
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5: //pushes first sample to observation deck
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park,true);
                    setPathState(6);
                }
                break;
            case 6: // drives to behind the second sample
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(wallPush, true);
                    setPathState(7);
                }
                break;
            case 7: // pushes the second sample into the observation deck
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(wallPush2,true);
                    setPathState(8);
                }
                break;

            case 8: // drives to be behind the third sample
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(wallPush3,true);
                    setPathState(9);
                }
                break;

            case 9: // pushes the third sample
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {


                    /* go home */




                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(put2,true);
                    setPathState(10);
                }



                break;

            case 10: // scores first specimen
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */


                if(!follower.isBusy()) {
                    /* go home */



                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(put3,true);
                    setPathState(11);
                }

                if (follower.getCurrentTValue() > 0.7){ //.3
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                }


                break;

            case 11: // drives back to observation deck
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(!follower.isBusy()) {
                    /* go home */


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(put4,true);
                    setPathState(12);


                }

                if (follower.getCurrentTValue() > 0.1){ //.3
                    linearMotor.setTargetPosition(armPickup);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                }

                break;

            case 12: // scores the second specimen
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(!follower.isBusy()) {
                    /* go home */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(put5,true);
                    setPathState(13);
                }

                if (follower.getCurrentTValue() > 0.7){
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                }
                break;

            case 13: // drives back to observation deck
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(!follower.isBusy()) {
                    /* go home */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(put6,true);
                    setPathState(14);
                }

                if (follower.getCurrentTValue() > 0.1){
                    linearMotor.setTargetPosition(armPickup);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                }
                break;

            case 14: // comes back to starting position
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(!follower.isBusy()) {
                    /* go home */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(home,true);
                    setPathState(15);
                }


                if (follower.getCurrentTValue() > 0.7){
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(moterSpeed);
                }
                break;


            case 15:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        linearMotor = hardwareMap.get(DcMotor.class, "lift");
        wrist = hardwareMap.get(Servo.class, "wrist"); // NAME WRIST SERVO CONFIGURATION TO wrist
        claw = hardwareMap.get(Servo.class, "claw"); // NAME CLAW SERVO CONFIGURATION TO claw
        armPickup = -1030;
        armClip = -390;
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("armEncoder", linearMotor.getCurrentPosition());
        telemetry.addData("currentTValue", follower.getCurrentTValue());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        /*
        linearMotor = hardwareMap.get(DcMotor.class, "lift");
        armPickup = -970;
        armClip = -700;
         */
        buildPaths();





    }

    /** This method is called continuously after Init while waiting for "play". **/
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

