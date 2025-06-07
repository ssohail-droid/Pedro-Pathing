package pedroPathing.AutoClass.Actuator;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorTicController {
    private DcMotor motor;
    private int highPos = 0;
    private int lowPos = 0;

    public MotorTicController(DcMotor motor) {
        this.motor = motor;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Set high and low tick positions
    public void setPositions(int low, int high) {
        this.lowPos = low;
        this.highPos = high;
    }

    public void goHigh() {
        motor.setTargetPosition(highPos);
        motor.setPower(1.0);
    }

    public void goLow() {
        motor.setTargetPosition(lowPos);
        motor.setPower(1.0);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }
}
