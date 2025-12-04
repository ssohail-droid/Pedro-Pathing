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
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

//@Autonomous(name = "BlueAUTOTEST DEC", group = "Examples")
public class BlueAUTOTEST extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final int Heading = 0;
    private final Pose onePos = new Pose(16.5, 35, Math.toRadians(90));
    private final Pose twoPos = new Pose(20, 31, Math.toRadians(73.4));
    private final Pose threePos = new Pose(66, 50, Math.toRadians(270));
    private final Pose fourPos = new Pose(66, 27, Math.toRadians(270));
    private final Pose fivePos = new Pose(90, 50, Math.toRadians(270));
    private final Pose sixPos = new Pose(90, 27, Math.toRadians(270));
    private final Pose sevenPos = new Pose(43, 18.5, Math.toRadians(90));


    private PathChain moveOne;
    private PathChain moveTwo;
    private PathChain moveThree;
    private PathChain moveFour;
    private PathChain moveFive;
    private PathChain moveSix;
    private PathChain moveSeven;
    private PathChain moveEight;
    private PathChain moveNine;







    // ------------------------------------------------------------

    public void buildPaths() {

        // Shoot here +3
        moveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(onePos), new Point(twoPos)))
                .setLinearHeadingInterpolation(onePos.getHeading(), twoPos.getHeading())
                .build();

        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twoPos), new Point(threePos)))
                .setLinearHeadingInterpolation(twoPos.getHeading(), threePos.getHeading())
                .build();

        moveThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(threePos), new Point(fourPos)))
                .setLinearHeadingInterpolation(threePos.getHeading(), fourPos.getHeading())
                .build();

        // Shoot here +3
        moveFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourPos), new Point(fivePos)))
                .setLinearHeadingInterpolation(fourPos.getHeading(), fivePos.getHeading())
                .build();

        moveFive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fivePos), new Point(twoPos)))
                .setLinearHeadingInterpolation(fivePos.getHeading(), twoPos.getHeading())
                .build();

        moveSix = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twoPos), new Point(fivePos)))
                .setLinearHeadingInterpolation(twoPos.getHeading(), fivePos.getHeading())
                .build();

        moveSeven = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fivePos), new Point(sixPos)))
                .setLinearHeadingInterpolation(fivePos.getHeading(), sixPos.getHeading())
                .build();

        moveEight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixPos), new Point(twoPos)))
                .setLinearHeadingInterpolation(sixPos.getHeading(), twoPos.getHeading())
                .build();

        moveNine = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twoPos), new Point(sevenPos)))
                .setLinearHeadingInterpolation(twoPos.getHeading(), sevenPos.getHeading())
                .build();
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveOne);

                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveTwo);

                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveThree);

                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveFour);

                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveFive);

                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveSix);

                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveSeven);

                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveEight);

                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveNine);

                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
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
            // alignStartTime = System.currentTimeMillis(); // Start timeout timer
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        // --- Alignment Telemetry ---
        if (pathState == 4 || pathState == 1) {
            telemetry.addLine("=== Alignment Sensors ===");
            telemetry.addData("Aligning", pathState == 4);
        }

        telemetry.update();
    }

    @Override
    public void stop() {

    }
}