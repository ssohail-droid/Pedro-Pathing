package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class LimeLightSensing extends LinearOpMode {

    private Robot BlueJai = new Robot(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        BlueJai.limelight.start();;
    }
}
