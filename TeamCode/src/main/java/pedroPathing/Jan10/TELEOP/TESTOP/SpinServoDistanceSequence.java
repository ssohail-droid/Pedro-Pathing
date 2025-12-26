package pedroPathing.Jan10.TELEOP.TESTOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Spin Servo Distance Sequence", group = "Test")
@Config
public class SpinServoDistanceSequence extends OpMode {

    // ===== DASHBOARD TUNABLE =====
    public static double DISTANCE_THRESHOLD_CM = 2.0;

    // Spin servo positions
    public static double SPIN_START = 0.20;
    public static double SPIN_MID   = 0.50;
    public static double SPIN_MAX   = 0.64;

    // CR servo power
    public static double CR_POWER = 1.0;

    // Cooldown to prevent double-trigger
    public static double DETECT_COOLDOWN_SEC = 0.5;

    // ===== HARDWARE =====
    private Servo spinServo;
    private DistanceSensor distanceSensor;
    private CRServo crLeft, crRight;

    // ===== STATE =====
    private int step = 0;
    private ElapsedTime cooldownTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spinServo = hardwareMap.get(Servo.class, "spin");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        crLeft  = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");

        crLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional extended PWM (safe for multi-turn)
        if (spinServo instanceof PwmControl) {
            ((PwmControl) spinServo)
                    .setPwmRange(new PwmControl.PwmRange(800, 2200));
        }

        // Initial positions
        spinServo.setPosition(SPIN_START);
        //crLeft.setPower(CR_POWER);
        //crRight.setPower(CR_POWER);

        cooldownTimer.reset();

        telemetry.addLine("Spin Servo Distance Sequence READY");
        telemetry.update();
    }

    @Override
    public void loop() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        boolean detected = distance <= DISTANCE_THRESHOLD_CM;

        // Advance sequence on detection (with cooldown)
        if (detected && cooldownTimer.seconds() > DETECT_COOLDOWN_SEC) {
            step++;
            cooldownTimer.reset();
        }

        // Clamp steps
        if (step > 2) step = 2;

        // Apply spin servo position
        switch (step) {
            case 0:
                spinServo.setPosition(SPIN_START);
                break;
            case 1:
                spinServo.setPosition(SPIN_MID);
                break;
            case 2:
                spinServo.setPosition(SPIN_MAX);
                break;
        }

        // CR servos always running
        crLeft.setPower(CR_POWER);
        crRight.setPower(CR_POWER);

        // ===== TELEMETRY =====
        telemetry.addData("Distance (cm)", "%.2f", distance);
        telemetry.addData("Detected", detected);
        telemetry.addData("Step", step);
        telemetry.addData("Spin Position",
                step == 0 ? SPIN_START :
                        step == 1 ? SPIN_MID :
                                SPIN_MAX);
        telemetry.addData("CR Power", CR_POWER);
        telemetry.update();
    }
}