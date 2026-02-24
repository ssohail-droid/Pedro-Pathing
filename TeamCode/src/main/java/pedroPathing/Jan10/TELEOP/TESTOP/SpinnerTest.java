package pedroPathing.Jan10.TELEOP.TESTOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester TeleOp", group = "TeleOp")
public class SpinnerTest extends OpMode {

    /* ================= HARDWARE ================= */
    private Servo spinServo;

    /* ================= POSITIONS ================= */
    // Intake Positions
    public static final double INTAKE_0 = 0.145;
    public static final double INTAKE_1 = 0.41;
    public static final double INTAKE_2 = 0.7;

    // Shooter Positions
    public static final double SHOOT_A = 0.56;
    public static final double SHOOT_B = 0.28;
    public static final double SHOOT_C = 0.00;

    @Override
    public void init() {
        // Hardware Map
        spinServo = hardwareMap.get(Servo.class, "spin");

        // Set initial position
        spinServo.setPosition(INTAKE_0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        /* ================= INTAKE CONTROLS ================= */
        // Press A for Intake 0
        if (gamepad1.a) {
            spinServo.setPosition(INTAKE_0);
        }
        // Press B for Intake 1
        else if (gamepad1.b) {
            spinServo.setPosition(INTAKE_1);
        }
        // Press X for Intake 2
        else if (gamepad1.x) {
            spinServo.setPosition(INTAKE_2);
        }

        /* ================= SHOOTER CONTROLS ================= */
        // Press Dpad Up for Shooter A
        if (gamepad1.dpad_up) {
            spinServo.setPosition(SHOOT_A);
        }
        // Press Dpad Right for Shooter B
        else if (gamepad1.dpad_right) {
            spinServo.setPosition(SHOOT_B);
        }
        // Press Dpad Down for Shooter C
        else if (gamepad1.dpad_down) {
            spinServo.setPosition(SHOOT_C);
        }

        /* ================= TELEMETRY ================= */
        telemetry.addData("Servo Position", spinServo.getPosition());
        telemetry.addLine("\nControls:");
        telemetry.addLine("A: Intake 0 | B: Intake 1 | X: Intake 2");
        telemetry.addLine("Up: Shoot A | Right: Shoot B | Down: Shoot C");
        telemetry.update();
    }
}
