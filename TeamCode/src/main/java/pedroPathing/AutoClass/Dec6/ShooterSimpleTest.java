package pedroPathing.AutoClass.Dec6;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Shooter Simple Test", group = "Tuning")
@Config
public class ShooterSimpleTest extends OpMode {

    // ======= Hardware =======
    private DcMotorEx shooterMotor, transferMotor, intakeMotor;
    private Servo gateServo, pusherServo;
    private DistanceSensor beamSensor;

    // ======= Configurable Values (Dashboard) =======
    public static double targetRPM = 2450;
    public static double kP = 0.1;
    public static double kI = 0.0003;
    public static double kD = 0.003;
    public static double kF = 0.000357;

    public static double GATE_OPEN = 0.6;
    public static double GATE_CLOSED = 0.3;
    public static double PUSHER_PUSH = 0.9;
    public static double PUSHER_RETRACT = 0.3;

    public static double BEAM_THRESHOLD_CM = 7.0;
    public static long PUSHER_DELAY_MS = 250;
    public static long RECOVERY_TIMEOUT_MS = 1000;

    private final double TICKS_PER_REV = 28.0;
    private ElapsedTime timer = new ElapsedTime();
    private String state = "START";

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        transferMotor = hardwareMap.get(DcMotorEx.class, "feed_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        gateServo = hardwareMap.get(Servo.class, "hold_servo");
        pusherServo = hardwareMap.get(Servo.class, "push_servo");
        beamSensor = hardwareMap.get(DistanceSensor.class, "shootDe");

        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        gateServo.setPosition(GATE_OPEN);
        pusherServo.setPosition(PUSHER_RETRACT);
    }

    @Override
    public void start() {
        shooterMotor.setVelocity(getTargetTPS());
        intakeMotor.setPower(1.0);
        transferMotor.setPower(1.0);
        state = "IDLE";
         timer.reset();
    }

    @Override
    public void loop() {
        double beamDist = beamSensor.getDistance(DistanceUnit.CM);
        boolean beamBroken = beamDist < BEAM_THRESHOLD_CM;
        double currentRPM = getShooterRPM();

// Update PIDF + velocity in real time
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        shooterMotor.setVelocity(getTargetTPS());

        switch (state) {
            case "IDLE":
                if (beamBroken) {
                    transferMotor.setPower(0);
                    gateServo.setPosition(GATE_CLOSED);
                    timer.reset();
                    state = "WAIT_FOR_RPM";
                }
                break;

            case "WAIT_FOR_RPM":
                if (isShooterReady() || timer.milliseconds() > RECOVERY_TIMEOUT_MS) {
                    gateServo.setPosition(GATE_OPEN);
                    transferMotor.setPower(1.0);
                    timer.reset();
                    state = "FEEDING";
                }
                break;

            case "FEEDING":
                if (beamBroken) {
                    transferMotor.setPower(0);
                    gateServo.setPosition(GATE_CLOSED);
                    timer.reset();
                    state = "PUSH";
                } else if (timer.milliseconds() > 1000) {
                    state = "PUSH";
                    transferMotor.setPower(0);
                    timer.reset();
                }
                break;

            case "PUSH":
                pusherServo.setPosition(PUSHER_PUSH);
                if (timer.milliseconds() > PUSHER_DELAY_MS) {
                    pusherServo.setPosition(PUSHER_RETRACT);
                    transferMotor.setPower(1.0);
                    gateServo.setPosition(GATE_OPEN);
                    state = "IDLE";
                }
                break;
        }

// ==== Telemetry ====
        telemetry.addData("State", state);
        telemetry.addData("RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Shooter TPS", shooterMotor.getVelocity());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Beam Distance (cm)", beamDist);
        telemetry.addData("Beam Broken", beamBroken);
        telemetry.update();
    }

    private double getShooterRPM() {
        return (shooterMotor.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    private double getTargetTPS() {
        return (targetRPM * TICKS_PER_REV) / 60.0;
    }

    private boolean isShooterReady() {
        return Math.abs(getShooterRPM() - targetRPM) < 100;
    }
}