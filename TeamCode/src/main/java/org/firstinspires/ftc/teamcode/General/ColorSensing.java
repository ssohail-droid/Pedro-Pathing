package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ColorSensing extends LinearOpMode {

    private RevColorSensorV3 colorSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get( RevColorSensorV3.class, "colorSensor");




    }
}
