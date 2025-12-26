package pedroPathing.Jan10.TELEOP.TESTOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Adjust Hood Servo Tuner", group = "Test")
@Config
public class AdjustHoodServoTuner extends OpMode {

    // ===== DASHBOARD CONTROLS =====
    public static double TARGET_POS = 0.50;   // move this in Dashboard
    public static double MIN_LIMIT  = 0.05;   // soft min
    public static double MAX_LIMIT  = 0.95;   // soft max

    // PWM range (good for 5-turn servos)
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    // ===== HARDWARE =====
    private Servo adjustServo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        adjustServo = hardwareMap.get(Servo.class, "adjust_servo");

        if (adjustServo instanceof PwmControl) {
            ((PwmControl) adjustServo)
                    .setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        }

        telemetry.addLine("Adjust Hood Servo Tuner READY");
        telemetry.addLine("Use FTC Dashboard → Config");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clamp to safe range
        double safePos = Math.max(MIN_LIMIT, Math.min(MAX_LIMIT, TARGET_POS));
        adjustServo.setPosition(safePos);

        telemetry.addData("Requested Pos", TARGET_POS);
        telemetry.addData("Applied Pos", safePos);
        telemetry.addData("Min Limit", MIN_LIMIT);
        telemetry.addData("Max Limit", MAX_LIMIT);
        telemetry.update();
    }
}