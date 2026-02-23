package pedroPathing.Talons.TeleOps;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.Talons.General.Robot;


import pedroPathing.Talons.General.SharedData;
import pedroPathing.Talons.General.PoseConstants;
import pedroPathing.Talons.General.ColorSensed;
import pedroPathing.Talons.General.Side;
@TeleOp
public class TeleOpTalons extends LinearOpMode {

    private Robot blueJai;

    boolean robotCentric;

    double speedMultiplier;
    int activeRPM;
    int ballsLaunched;
    boolean launching;
    Timer launchTimer;

    static int closeLaunch = 2640, farLaunch = 4100, REST;

    private Follower f;


    public void runOpMode() throws InterruptedException{
        blueJai = new Robot(hardwareMap);

        f.setStartingPose(new Pose(0,0,0));
        f.update();

        waitForStart();

        f.startTeleopDrive();
        while (opModeIsActive()){
            f.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y * speedMultiplier,
                    -gamepad1.left_stick_x * speedMultiplier,
                    -gamepad1.right_stick_x * speedMultiplier,
                    robotCentric
            );




            if (gamepad1.left_trigger >= .2){
                speedMultiplier = .2;
            }
            else speedMultiplier = 1;

            if(gamepad2.right_bumper)
                blueJai.startIntake(true);
            else if(gamepad2.left_bumper)
                blueJai.startIntake(false);
            else
                blueJai.stopIntake();

            if (gamepad1.dpad_up && SharedData.getGreenIndex() != -1){
                blueJai.setStoragePos(SharedData.getGreenIndex() , false);
                blueJai.setKickServo(true);
                blueJai.setLaunchVelocity(activeRPM);
            }
            else if (gamepad1.dpad_down && SharedData.getPurpleIndex() != -1){
                blueJai.setStoragePos(SharedData.getPurpleIndex() , false);
                blueJai.setKickServo(true);
                blueJai.setLaunchVelocity(activeRPM);
            }



        }


    }
    public void updateLaunch(){
        if (launching && blueJai.shooterAtSpeed()) {
            if (launchTimer.getElapsedTimeSeconds() > 0.35 /* && kicker is at not at launch pos*/) {
                //launch
                blueJai.setLaunchVelocity(closeLaunch);
            }
            if (launchTimer.getElapsedTimeSeconds() > 0.75) {
                //reset kicker
            }
            if (launchTimer.getElapsedTimeSeconds() > 1.25) {
                ballsLaunched++; // count to make sure it knows how many its shot
                launchTimer.resetTimer();
            }

            if (ballsLaunched > 3) {
                launching = false;
                ballsLaunched = 0;
                blueJai.setLaunchVelocity(REST);
            }
        }

    }



}