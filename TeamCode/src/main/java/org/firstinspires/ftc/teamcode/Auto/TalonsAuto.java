package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.General.Robot;

@Autonomous
public class TalonsAuto extends OpMode {
    private int index;
    private Robot BlueJai;
    private Limelight3A limelight;

    @Override
    public void init() {
        BlueJai = new Robot(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class , "limelight");
        limelight.start();

    }

    @Override
    public void init_loop() {

        int ID = figureID();
        if (ID == 21) index = 0;
        else if (ID == 22) index = 1;
        else if (ID == 23) index = 2;

    }

    @Override
    public void loop() {

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
}
