package pedroPathing.State;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
//@TeleOp(name="intakeAndTranTEST")

public class intakeAndTran extends OpMode {

    DcMotorEx upPipe;
    DcMotorEx midPipe;
    DcMotorEx intake;


    Servo gateServo;
    Servo hoodServo;

    private DcMotorEx shooter;

    // Toggle variables
    boolean shooterToggle = false;
    boolean lastButton = false;

    // Dashboard adjustable RPM
    public static double SHOOT_RPM = 2600;

    // Shooter PID constants
    public static double SHOOTER_P = 70;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 8;
    public static double SHOOTER_F = 13.5;

    private static final double TICKS_PER_REV = 28.0;


    // Dashboard adjustable power
    public static double upPipePower = 1;
    public static double midPipePower = 1;
    public static double intakePower = 1;

    @Override
    public void init() {

        upPipe = hardwareMap.get(DcMotorEx.class, "upPipe");
        midPipe = hardwareMap.get(DcMotorEx.class, "midPipe");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "stop");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        upPipe.setDirection(DcMotorSimple.Direction.REVERSE);
        midPipe.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        gateServo.setPosition(0.4);

        shooter = hardwareMap.get(DcMotorEx.class, "shoot");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F)
        );
    }

    @Override
    public void loop() {

        // Toggle button (gamepad2 Y)
        boolean button = gamepad2.y;
        if(button && !lastButton){
            shooterToggle = !shooterToggle;
            hoodServo.setPosition(0.75);
        }
        lastButton = button;

// Shooter control
        if(shooterToggle){
            shooter.setVelocity((SHOOT_RPM * TICKS_PER_REV) / 60.0);
        } else {
            shooter.setVelocity(0);
        }



// Preset: upPipe + midPipe
        if(gamepad1.left_bumper){
            midPipe.setPower(midPipePower);
            intake.setPower(intakePower);
            gateServo.setPosition(0.2);
            hoodServo.setPosition(0.75);
        }

// Preset: all motors
        if(gamepad1.right_bumper){
            upPipe.setPower(upPipePower);
            midPipe.setPower(midPipePower);
            intake.setPower(intakePower);
            gateServo.setPosition(0.4);
            hoodServo.setPosition(0.75);
        }

// Stop everything
        if(gamepad1.x){
            upPipe.setPower(0);
            midPipe.setPower(0);
            intake.setPower(0);
        }

        telemetry.addData("upPipe Power", upPipe.getPower());
        telemetry.addData("midPipe Power", midPipe.getPower());
        telemetry.addData("intake Power", intake.getPower());
        double currentRPM = shooter.getVelocity() * 60.0 / TICKS_PER_REV;
        telemetry.addData("Shooter Toggle", shooterToggle);
        telemetry.addData("Target RPM", SHOOT_RPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.update();
    }
}