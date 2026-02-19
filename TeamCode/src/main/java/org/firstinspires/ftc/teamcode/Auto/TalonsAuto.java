package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.General.PoseConstants;
import org.firstinspires.ftc.teamcode.General.Robot;
import org.firstinspires.ftc.teamcode.General.SharedData;
import org.firstinspires.ftc.teamcode.General.Side;

@Autonomous
public class TalonsAuto extends OpMode {
    private Robot blueJai;
    private Follower f = null;
    private Limelight3A limelight;

    private int pathState;
    private Timer pathTimer,opmodeTimer,launchTimer,detectColorTimer;

    private PoseConstants poses = new PoseConstants();
    private PathChain toShootOne, toAlignOne, toPickupOne, toShootTwo, toAlignTwo, toPickupTwo, toShootThree, leave;

    private int timesLaunched = 0;

    @Override
    public void init() {
        blueJai = new Robot(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class , "limelight");
        limelight.start();
        SharedData.reset();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        launchTimer = new Timer();
        detectColorTimer = new Timer();

        f.setStartingPose(SharedData.side == Side.RED ? new Pose(111,135,0) : new Pose(32, 135, Math.toRadians(180)));
        buildPaths();


    }

    @Override
    public void init_loop() {

        int ID = figureID();
        if (ID == 21) SharedData.greenIndex = 0;
        else if (ID == 22) SharedData.greenIndex = 1;
        else if (ID == 23) SharedData.greenIndex = 2;

    }

    @Override
    public void loop() {
        autoPathUpdates();
    }

    public int figureID(){
        LLResult result = limelight.getLatestResult();
        try{

            telemetry.addData("ID" , result.getFiducialResults().get(0).getFiducialId());
            return result.getFiducialResults().get(0).getFiducialId();
        }
        catch (Exception e){
            telemetry.addData("ID" , "not found");
            return -1;
        }
    }
    void buildPaths(){
        toShootOne = f.pathBuilder()
                .addPath(new BezierLine(poses.startPos, poses.shootPos))
                .setLinearHeadingInterpolation(poses.startPos.getHeading(), poses.shootPos.getHeading())
                .build();

        toAlignOne = f.pathBuilder()
                .addPath(new BezierLine(poses.shootPos, poses.alignOne))
                .setLinearHeadingInterpolation(poses.shootPos.getHeading(), poses.alignOne.getHeading())
                .build();
        toPickupOne = f.pathBuilder()
                .addPath(new BezierLine(poses.alignOne, poses.pickupOne))
                .setLinearHeadingInterpolation(poses.alignOne.getHeading(), poses.pickupOne.getHeading())
                .build();

        toShootTwo = f.pathBuilder()
                .addPath(new BezierLine(poses.pickupOne, poses.shootPos))
                .setLinearHeadingInterpolation(poses.pickupOne.getHeading(), poses.shootPos.getHeading())
                .build();
        toAlignTwo = f.pathBuilder()
                .addPath(new BezierLine(poses.shootPos, poses.alignTwo))
                .setLinearHeadingInterpolation(poses.shootPos.getHeading(), poses.alignTwo.getHeading())
                .build();
        toPickupTwo = f.pathBuilder()
                .addPath(new BezierLine(poses.alignTwo, poses.pickupTwo))
                .setLinearHeadingInterpolation(poses.alignTwo.getHeading(), poses.pickupTwo.getHeading())
                .build();
        toShootThree = f.pathBuilder()
                .addPath(new BezierLine(poses.pickupTwo, poses.shootPos))
                .setLinearHeadingInterpolation(poses.pickupTwo.getHeading(), poses.shootPos.getHeading())
                .build();
        leave = f.pathBuilder()
                .addPath(new BezierLine(poses.shootPos, poses.leave))
                .setLinearHeadingInterpolation(poses.shootPos.getHeading(), poses.leave.getHeading())
                .build();
    }
    void autoPathUpdates(){
        sendPose();
        switch (pathState){
            case 0:
                f.followPath(toShootOne,true);
                setPathState(1);
                break;
            case 1:
                if(!f.isBusy() && SharedData.isEmpty()){
                    f.followPath(toAlignOne,true);
                    setPathState(2);
                    //launching false
                    blueJai.stopRollers();
                }
                else if(!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    //launching true
                    blueJai.startRollers();
                }
                break;
            case 2:
                if(!f.isBusy()){
                    f.followPath(toPickupOne,true);
                    f.setMaxPower(.5);
                    blueJai.startIntake(true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!f.isBusy() || SharedData.isFull()){
                    f.followPath(toShootTwo,true);
                    f.setMaxPower(1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!f.isBusy() && SharedData.isEmpty()){
                    f.followPath(toAlignTwo,true);
                    setPathState(5);
                    //launching false
                    blueJai.stopRollers();
                }
                else if(!f.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                    //launching true
                    blueJai.stopIntake();
                    blueJai.startRollers();
                }
                break;
            case 5:
                if(!f.isBusy()){
                    f.followPath(toPickupTwo,true);
                    f.setMaxPower(.5);
                    blueJai.startIntake(true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!f.isBusy() || SharedData.isFull()){
                    f.followPath(toShootThree);
                    f.setMaxPower(1);
                    setPathState(7);
                }
                break;
            case 7:
                if(!f.isBusy() && SharedData.isEmpty() && opmodeTimer.getElapsedTimeSeconds() < 28.5){
                    f.followPath(leave);
                    setPathState(8);
                    blueJai.startRollers();
                }
                else if(!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    //launching true
                    blueJai.stopIntake();
                    blueJai.startRollers();
                }
                break;
            case 8:
                if(!f.isBusy()){
                    setPathState(-1);
                    //launching false
                }
                break;

        }
    }
    public void sendPose(){SharedData.toTeleopPose = f.getPose();}
    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();

    }
}
