package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    DcMotorEx intake, shooter;
    CRServo crRight, crLeft;
    Servo spinServo, kickServo, adjustServo;
    ColorSensor color;
    DistanceSensor distance;
    public Robot(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake = hardwareMap.get(DcMotorEx.class, "shooter");

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

        distance = hardwareMap.get(DistanceSensor.class, "sensor_color_left");
        color = hardwareMap.get(ColorSensor.class, "sensor_color_right");
    }
}
