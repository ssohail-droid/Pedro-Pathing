package pedroPathing.SubSystem.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.SubSystem.ServoSubsystem;

@TeleOp(name="Servo Subsystem OpMode", group="Subsystems")
public class ServoSubsystemOpMode extends LinearOpMode {
    private ServoSubsystem servos;

    @Override
    public void runOpMode() {
        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");
        servos = new ServoSubsystem(holdServo, pushServo);

        telemetry.addLine("Servos ready.");
        telemetry.addLine("A = Engage hold");
        telemetry.addLine("B = Release hold");
        telemetry.addLine("X = Engage push");
        telemetry.addLine("Y = Retract push");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // A: Engage hold
            if (gamepad1.a) {
                servos.engageHold();
            }

            // B: Release hold
            if (gamepad1.b) {
                servos.releaseHold();
            }

            // X: Engage push
            if (gamepad1.x) {
                servos.engagePush();
            }

            // Y: Retract push
            if (gamepad1.y) {
                servos.retractPush();
            }

            telemetry.addLine("Use buttons to control servos");
            telemetry.update();
        }
    }
}
