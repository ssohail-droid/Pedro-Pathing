package org.firstinspires.ftc.teamcode.pedroPathing.Jan10.TELEOP.TESTOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Spin Servo Dashboard Tuner", group = "Test")
@Config
public class SpinServoDashboard extends OpMode {

    // ===== DASHBOARD VALUE =====
    public static double SPIN_POSITION = 0.5; // Adjust live in Dashboard (0.0–1.0)

    // ===== HARDWARE =====
    private Servo spinServo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spinServo = hardwareMap.get(Servo.class, "spin");

        // Optional: extended PWM range (only if your servo supports it)
        if (spinServo instanceof PwmControl) {
            ((PwmControl) spinServo)
                    .setPwmRange(new PwmControl.PwmRange(800, 2200));
        }

        telemetry.addLine("Spin Servo Dashboard Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        spinServo.setPosition(SPIN_POSITION);

        telemetry.addData("Spin Servo Pos", SPIN_POSITION);
        telemetry.update();
    }
}