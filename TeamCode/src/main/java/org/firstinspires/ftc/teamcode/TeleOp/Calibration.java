package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.General.SharedData;
import org.firstinspires.ftc.teamcode.General.Side;

@TeleOp
public class Calibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        SharedData.reset();
        while(opModeIsActive())
        {
            if(gamepad1.a || gamepad2.a)
                SharedData.side = Side.BLUE;
            else if(gamepad1.b || gamepad2.b)
                SharedData.side = Side.RED;
            telemetry.addLine("B to set to red\nA to set to blue");
            telemetry.addData("Side", SharedData.side);
            telemetry.update();
        }

    }
}