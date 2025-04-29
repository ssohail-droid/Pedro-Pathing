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

@Autonomous(name = "Specimen Auto (V10.6)", group = "Examples")
public class StateMachineWithPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final int Heading = 180;
    private final int SamplePushWall = 20;

    DcMotor linearMotor; // variables must be created here but initialized in loop()
    Servo wrist;
    Servo claw;


    private int armPickup;
    private int armClip;
    private int armClipDown;


    private double motorSpeed;


    private double unitOnePathSpeed;
    private double unitTwoPathSpeed;
    private double unitThreePathSpeed;
    private double unitFourPathSpeed;

    private boolean unitTestOne;
    private boolean unitTestTwo;
    private boolean unitTestThree;
    private  boolean unitTestFour;



    private final Pose startPos = new Pose(9, 72, Math.toRadians(Heading));
    private final Pose SubmersiblePos = new Pose(33.5, 72, Math.toRadians(Heading));
    private final Pose Submersible2Pos = new Pose(39, 72, Math.toRadians(Heading));
    private final Pose SubmersibleBackPos = new Pose(24, 72, Math.toRadians(Heading));


    private final Pose clearBracePos = new Pose(24, 35.1, Math.toRadians(Heading));
    private final Pose behindSamplePos = new Pose(60, 35.1, Math.toRadians(Heading));
    private final Pose behindSampleToPushPos = new Pose(60, 24, Math.toRadians(Heading));



    private final Pose pushColourSampleOnePos = new Pose(SamplePushWall, 24, Math.toRadians(Heading));
    private final Pose returnToSampleTwoForPushPos = new Pose(60, 13, Math.toRadians(Heading));
    private final Pose returnToSampleTwoForPushControlPoint = new Pose(60.968698517298186, 35.82207578253706, Math.toRadians(Heading));
    //

    private final Pose pushColourSampleTwoPos = new Pose(SamplePushWall, 13, Math.toRadians(Heading));
    private final Pose returnToSampleThreeForPushPos = new Pose(60, 9.5, Math.toRadians(Heading));//need to change to (0,)
    private final Pose returnToSampleThreeForPushControlPoint = new Pose(55.98682042833608, 18.50411861614498, Math.toRadians(Heading));

    private final Pose pushColourSampleThreePos = new Pose(SamplePushWall, 9.5, Math.toRadians(Heading));

    private final Pose specimenPickUpPos = new Pose(17, 28, Math.toRadians(Heading));
    private final Pose specimenPickUpControlPoint = new Pose(23.72322899505766, 21.113673805601323, Math.toRadians(Heading));
    private final Pose specimenPickUpAdjustPos = new Pose(10, 28, Math.toRadians(Heading));


    /*Bezier line*/
    private PathChain moveToSubmersible;
    private PathChain moveToSubmersible2;
    private PathChain moveBackFromSubmersible;
    private PathChain moveClearBrace;
    private PathChain moveToBehindSample;
    private PathChain moveBehindSampleToPush;
    private PathChain movePushColourSampleOne;
    private PathChain movePushColourSampleTwo;
    private PathChain movePushColourSampleThree;
    private PathChain moveSpecimenPickUpAdjust;
    private PathChain moveScoreSpecimen;
    private PathChain moveGetScoreSpecimen;

    /*Bezier curve*/
    private Path moveToPushColourSampleTwo;
    private Path moveToPushColourSampleThree;
    private Path moveToSpecimenPickUp;

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

        moveToPushColourSampleTwo = new Path(new BezierCurve(new Point(pushColourSampleOnePos), /* Control Point */ new Point(returnToSampleTwoForPushControlPoint), new Point(returnToSampleTwoForPushPos)));
        moveToPushColourSampleTwo.setLinearHeadingInterpolation(pushColourSampleOnePos.getHeading(), returnToSampleTwoForPushPos.getHeading());

        movePushColourSampleTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(returnToSampleTwoForPushPos), new Point(pushColourSampleTwoPos)))
                .setLinearHeadingInterpolation(returnToSampleTwoForPushPos.getHeading(), pushColourSampleTwoPos.getHeading())
                .build();

        moveToPushColourSampleThree = new Path(new BezierCurve(new Point(pushColourSampleTwoPos), /* Control Point */ new Point(returnToSampleThreeForPushControlPoint), new Point(returnToSampleThreeForPushPos)));
        moveToPushColourSampleThree.setLinearHeadingInterpolation(pushColourSampleTwoPos.getHeading(), returnToSampleThreeForPushPos.getHeading());


        movePushColourSampleThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(returnToSampleThreeForPushPos), new Point(pushColourSampleThreePos)))
                .setLinearHeadingInterpolation(returnToSampleThreeForPushPos.getHeading(), pushColourSampleThreePos.getHeading())
                .build();

        moveToSpecimenPickUp = new Path(new BezierCurve(new Point(pushColourSampleThreePos), /* Control Point */ new Point(specimenPickUpControlPoint), new Point(specimenPickUpPos)));
        moveToSpecimenPickUp.setLinearHeadingInterpolation(pushColourSampleThreePos.getHeading(), specimenPickUpPos.getHeading());

        moveSpecimenPickUpAdjust = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickUpPos), new Point(specimenPickUpAdjustPos)))
                .setLinearHeadingInterpolation(specimenPickUpPos.getHeading(), specimenPickUpAdjustPos.getHeading())
                .build();


        moveScoreSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickUpAdjustPos), new Point(SubmersiblePos)))
                .setLinearHeadingInterpolation(specimenPickUpAdjustPos.getHeading(), SubmersiblePos.getHeading())
                .build();


        moveGetScoreSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SubmersiblePos), new Point(specimenPickUpPos)))
                .setLinearHeadingInterpolation(SubmersiblePos.getHeading(), specimenPickUpPos.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {

                    /*end of unit test one*/
            case 0:

                if (follower.getCurrentTValue() > 0.0){
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);
                    claw.setPosition(0.19);
                    wrist.setPosition(0.7);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(unitOnePathSpeed);
                    follower.followPath(moveToSubmersible, unitTestOne);
                    setPathState(1);
                }

                break;
            case 1:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitOnePathSpeed);
                    follower.followPath(moveToSubmersible2, unitTestOne);

                    setPathState(2);

                }
                if (follower.getCurrentTValue() > 0.9){
                    linearMotor.setTargetPosition(armClipDown);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);

                }

                break;


            case 2:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitOnePathSpeed);
                    follower.followPath(moveBackFromSubmersible, unitTestOne);

                    setPathState(3);
                }
                if (follower.getCurrentTValue()>0.7){
                    claw.setPosition(0.0);
                }

                break;

                    /*All above is unit test 1.*/

                    /*End of unit test 2.*/

            case 3:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitTwoPathSpeed);
                    follower.followPath(moveClearBrace, unitTestTwo);


                    setPathState(4);
                }



                if (follower.getCurrentTValue() > 0.5){
                    linearMotor.setTargetPosition(armPickup);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);

                    claw.setPosition(0.0);
                    wrist.setPosition(0.0);
                }




                break;

            case 4:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitTwoPathSpeed);
                    follower.followPath(moveToBehindSample, unitTestTwo);

                    setPathState(5);
                }
                break;

            case 5:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitTwoPathSpeed);
                    follower.followPath(moveBehindSampleToPush, unitTestTwo);
                    setPathState(6);
                }
                break;

            case 6:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitThreePathSpeed);
                    follower.followPath(movePushColourSampleOne);
                    setPathState(7);
                }
                break;

                        /*All above is unit test 2.*/

                        /*End of unit test 3.*/

            case 7:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitThreePathSpeed);
                    follower.followPath(moveToPushColourSampleTwo, unitTestThree);
                    setPathState(8);
                }
                break;

            case 8:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitThreePathSpeed);
                    follower.followPath(movePushColourSampleTwo, unitTestThree);
                    setPathState(9);
                }
                break;


            case 9:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitThreePathSpeed);
                    follower.followPath(moveToPushColourSampleThree, unitTestThree);
                    setPathState(10);
                }
                break;

            case 10:


                if(!follower.isBusy()) {

                    follower.setMaxPower(unitThreePathSpeed);
                    follower.followPath(movePushColourSampleThree, unitTestThree);
                    setPathState(11);
                }
                break;
                        /*All above is unit test 3.*/

                        /*End of unit test 4.*/

            case 11:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitFourPathSpeed);
                    follower.followPath(moveToSpecimenPickUp, unitTestFour);
                    setPathState(12);

                }
                break;

            /*End of unit 4.1*/

            case 12:

                if(!follower.isBusy()) {

                    follower.setMaxPower(0.5);
                    follower.followPath(moveSpecimenPickUpAdjust, unitTestFour);
                    setPathState(13);
                }

                if (follower.getCurrentTValue()>0.4){
                    claw.setPosition(0.18);

                }




                break;




            case 13:







                if(!follower.isBusy()) {

                   follower.setMaxPower(unitFourPathSpeed);
                   follower.followPath(moveScoreSpecimen,unitTestFour);




                    setPathState(14);
                }





                if (follower.getCurrentTValue()>0.5){
                    wrist.setPosition(0.7);
                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);

                }





                break;

            case 14:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitOnePathSpeed);
                    follower.followPath(moveToSubmersible2, unitTestFour);

                    setPathState(15);

                }
                if (follower.getCurrentTValue() > 0.88){
                    claw.setPosition(0.20);
                    linearMotor.setTargetPosition(armClipDown);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);

                }


                break;



            case 15:

                if(!follower.isBusy()) {
                    //follower.setMaxPower(unitFourPathSpeed);
                    //follower.followPath(moveGetScoreSpecimen);



                    setPathState(-16);
                }


                break;

            case 16:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitFourPathSpeed);
                    follower.followPath(moveSpecimenPickUpAdjust);

                    setPathState(17);
                }

                if (follower.getCurrentTValue()>0.2){

                    claw.setPosition(0.0);
                    wrist.setPosition(0.0);

                    linearMotor.setTargetPosition(armPickup);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);


                }
                break;

            case 17:

                if(!follower.isBusy()) {

                    follower.setMaxPower(0.19);
                    follower.followPath(moveSpecimenPickUpAdjust, unitTestFour);
                    setPathState(18);
                }
                break;
///
            case 18:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitFourPathSpeed);
                    follower.followPath(moveScoreSpecimen,unitTestFour);



                    setPathState(19);
                }
                if (follower.getCurrentTValue()>0.0){
                    claw.setPosition(0.17);
                    wrist.setPosition(0.7);

                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);


                }
                break;

            case 19:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitOnePathSpeed);
                    follower.followPath(moveToSubmersible2, unitTestFour);

                    setPathState(20);

                }
                if (follower.getCurrentTValue() > 0.88){
                    claw.setPosition(0.20);
                    linearMotor.setTargetPosition(armClipDown);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);

                }


                break;

                /*end of unit 4.2*/




            case 20:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitFourPathSpeed);
                    follower.followPath(moveScoreSpecimen,unitTestFour);




                    setPathState(21);
                }
                if (follower.getCurrentTValue()>0.0){
                    claw.setPosition(0.17);
                    wrist.setPosition(0.7);

                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);


                }
                break;

            case 21:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitOnePathSpeed);
                    follower.followPath(moveToSubmersible2, unitTestFour);

                    setPathState(22);

                }
                if (follower.getCurrentTValue() > 0.88){
                    claw.setPosition(0.20);
                    linearMotor.setTargetPosition(armClipDown);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);

                }


                break;



            case 22:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitFourPathSpeed);
                    follower.followPath(moveGetScoreSpecimen);



                    setPathState(23);
                }


                break;

            case 23:

                if(!follower.isBusy()) {
                    follower.setMaxPower(unitFourPathSpeed);
                    follower.followPath(moveSpecimenPickUpAdjust);

                    setPathState(24);
                }

                if (follower.getCurrentTValue()>0.2){

                    claw.setPosition(0.0);
                    wrist.setPosition(0.0);

                    linearMotor.setTargetPosition(armPickup);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);


                }
                break;
///
            case 24:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitFourPathSpeed);
                    follower.followPath(moveScoreSpecimen,unitTestFour);



                    setPathState(25);
                }
                if (follower.getCurrentTValue()>0.0){
                    claw.setPosition(0.17);
                    wrist.setPosition(0.7);

                    linearMotor.setTargetPosition(armClip);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);


                }
                break;

            case 25:

                if(!follower.isBusy()) {

                    follower.setMaxPower(unitOnePathSpeed);
                    follower.followPath(moveToSubmersible2, unitTestFour);


                    setPathState(26);

                }
                if (follower.getCurrentTValue() > 0.88){
                    claw.setPosition(0.20);
                    linearMotor.setTargetPosition(armClipDown);
                    linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearMotor.setPower(motorSpeed);

                }


                break;


            case 26:

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

        linearMotor = hardwareMap.get(DcMotor.class, "lift");
        linearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(0.19);
        wrist.setPosition(0.7);




    }

    @Override
    public void loop() {

        //linearMotor = hardwareMap.get(DcMotor.class, "lift");
        //wrist = hardwareMap.get(Servo.class, "wrist");
       // claw = hardwareMap.get(Servo.class, "claw");

        armPickup = -1053;
        armClip = -425;
        armClipDown =-370;


        motorSpeed = 0.67;


        unitOnePathSpeed = 0.4;
        unitTwoPathSpeed = 0.4;
        unitThreePathSpeed = 0.4;
        unitFourPathSpeed = 0.4;



        unitTestOne = true;
        unitTestTwo = false;
        unitTestThree = false;
        unitTestFour = false;



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
