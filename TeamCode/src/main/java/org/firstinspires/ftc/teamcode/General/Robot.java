package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {
    DcMotorEx intake, shooter;
    CRServo crRight, crLeft;
    Servo spinServo, kickServo, adjustServo;
    ColorSensor color;
    DistanceSensor distance;

    int slotGoal;
    Limelight3A limelight;

    public Robot(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(70, 0, 8, 13.5)
        );

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo = hardwareMap.get(Servo.class, "spin");
        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        adjustServo = hardwareMap.get(Servo.class, "adjust_servo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        distance = hardwareMap.get(DistanceSensor.class, "sensor_color_left");
        color = hardwareMap.get(ColorSensor.class, "sensor_color_right");
    }

    void startIntake(boolean in){
        crRight.setPower(1);
        crLeft.setPower(1);
        intake.setPower(in ? .9 : -.9);
    }
    void stopIntake(){
        crRight.setPower(0);
        crLeft.setPower(0);
        intake.setPower(0);
    }

    boolean detectBall(){
        return distance.getDistance(DistanceUnit.CM) < 3;
    }

    ColorSensed detectColor(){
        return ColorSensed.INCONCLUSIVE;
    }

    void setStoragePos(int slot, boolean intake){
        slotGoal = slot;
        if (intake){
            if(slot == 1){spinServo.setPosition(.145);}
            else if (slot == 2){spinServo.setPosition(.41);}
            else if (slot == 3){spinServo.setPosition(.7);}
        }else{
            if(slot == 1){spinServo.setPosition(.56);}
            else if (slot == 2){spinServo.setPosition(.28);}
            else if (slot == 3){spinServo.setPosition(0);}
        }
    }

    void setKickServo(boolean kick){
        if(kick){kickServo.setPosition(.7);}
        else{kickServo.setPosition(1);}
    }

    void setAdjustServo(){
        //figure out positions from sohail
        //lowest value is closest
    }


}
