package org.firstinspires.ftc.teamcode.TeleOp;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.General.Constants;
import org.firstinspires.ftc.teamcode.General.Robot;
import org.firstinspires.ftc.teamcode.General.SharedData;
import org.firstinspires.ftc.teamcode.General.Side;

public class TeleOpTalons extends LinearOpMode {

    private Robot BlueJai;

    boolean robotCentric;

    double speedMultiplier;


    private Follower f;


    public void runOpMode() throws InterruptedException{
        BlueJai = new Robot(hardwareMap);

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(new Pose(0,0,0));
        f.update();

        waitForStart();

        f.startTeleOpDrive(true);
        while (opModeIsActive()){
            f.setTeleOpDrive(
                    -gamepad1.left_stick_y * speedMultiplier,
                    -gamepad1.left_stick_x * speedMultiplier,
                    -gamepad1.right_stick_x * speedMultiplier,
                    robotCentric,
                    (SharedData.side == Side.RED || robotCentric) ? 0 : Math.toRadians(180)
            );


            if (gamepad1.left_trigger >= .2){
                speedMultiplier = .2;
            }
            else speedMultiplier = 1;

            if(SharedData.isFull())
                BlueJai.startIntake(false);
            else if(gamepad1.right_bumper)
                BlueJai.startIntake(true);
            else if(gamepad1.left_bumper)
                BlueJai.startIntake(false);
            else
                BlueJai.stopIntake();


        }

    }



}
