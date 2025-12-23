package pedroPathing.AutoTeleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Intake Ball Counter", group = "Main")
@Config
public class IntakeBallCounter extends OpMode {

    // ===== HARDWARE =====
    private DcMotorEx intake;
    private CRServo crLeft, crRight;
    private Servo spinServo;
    private DistanceSensor distanceSensor;

    // ===== TUNABLES =====
    public static double INTAKE_POWER = 0.4;
    public static double CR_POWER = 1.0;
    public static double DISTANCE_THRESHOLD_CM = 2.0;

    public static double SPIN_0 = 0.18;
    public static double SPIN_1 = 0.46;
    public static double SPIN_2 = 0.63;

    public static double DETECT_COOLDOWN_SEC = 0.5;

    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    // ===== STATE =====
    private int ballCount = 0;
    private boolean lastDetected = false;
    private final ElapsedTime cooldownTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo = hardwareMap.get(Servo.class, "spin");
        if (spinServo instanceof PwmControl) {
            ((PwmControl) spinServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        }

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        crLeft.setPower(0);
        crRight.setPower(0);
        intake.setPower(0);

        spinServo.setPosition(SPIN_0);
        cooldownTimer.reset();
    }

    @Override
    public void loop() {

        // ----- Distance detection -----
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        boolean detected = distance <= DISTANCE_THRESHOLD_CM;

        // Rising-edge detection with cooldown
        if (detected && !lastDetected && cooldownTimer.seconds() > DETECT_COOLDOWN_SEC) {
            if (ballCount < 2) {   // cap at 2 for now
                ballCount++;
            }
            cooldownTimer.reset();
        }
        lastDetected = detected;

        // ----- Apply state outputs -----
        intake.setPower(INTAKE_POWER);
        crLeft.setPower(CR_POWER);
        crRight.setPower(CR_POWER);

        switch (ballCount) {
            case 0:
                spinServo.setPosition(SPIN_0);
                break;
            case 1:
                spinServo.setPosition(SPIN_1);
                break;
            case 2:
                spinServo.setPosition(SPIN_2);
                break;
        }

        // ----- Telemetry -----
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Distance (cm)", "%.2f", distance);
        telemetry.addData("Detected", detected);
        telemetry.addData("Spin Pos", spinServo.getPosition());
        telemetry.update();
    }
}
