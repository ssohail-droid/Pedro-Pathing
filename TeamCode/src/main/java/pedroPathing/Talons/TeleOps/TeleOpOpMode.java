package pedroPathing.Talons.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Talons.General.Robot;
import pedroPathing.Talons.General.SharedData;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class TeleOpOpMode extends OpMode {



    private Robot blueJai;

    boolean robotCentric;

    double speedMultiplier;
    int activeRPM;
    int ballsLaunched;
    boolean launching;
    Timer launchTimer;

    static int closeLaunch = 2640, farLaunch = 4100, REST;

    private Follower f;

    @Override
    public void init() {
        blueJai = new Robot(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        f = new Follower(hardwareMap);
        f.setStartingPose(new Pose(0,0,0));
        f.update();
        f.startTeleopDrive();

    }

    @Override
    public void loop() {


        f.setTeleOpMovementVectors(
                -gamepad1.left_stick_y * speedMultiplier,
                -gamepad1.left_stick_x * speedMultiplier,
                -gamepad1.right_stick_x * speedMultiplier,
                robotCentric
        );
        f.update();


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
