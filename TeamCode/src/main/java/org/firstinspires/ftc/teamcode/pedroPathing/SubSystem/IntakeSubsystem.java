package org.firstinspires.ftc.teamcode.pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeSubsystem {
    private final DcMotor intakeMotor;
    private double intakePower = 1.0; // configurable

    public IntakeSubsystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void setPower(double power) {
        this.intakePower = Math.max(-1.0, Math.min(1.0, power));
        // If running, update to new power
        if (Math.abs(intakeMotor.getPower()) > 1e-6) { //1e-6 means 0.000001
            intakeMotor.setPower(-intakePower);
        }
    }

    public void start() {
        intakeMotor.setPower(-intakePower);
    }

    public void reverse() {
        intakeMotor.setPower(intakePower);
    }

    public void stop() {
        intakeMotor.setPower(0.0);
    }

    public boolean isRunning() {
        return Math.abs(intakeMotor.getPower()) > 1e-6; //
    }
}