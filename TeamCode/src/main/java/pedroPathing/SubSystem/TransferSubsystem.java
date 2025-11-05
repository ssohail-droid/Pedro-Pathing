package pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TransferSubsystem {
    private final DcMotor feedMotor;
    private double feedPower = 1.0; // configurable

    public TransferSubsystem(DcMotor feedMotor) {
        this.feedMotor = feedMotor;
        this.feedMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.feedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void setPower(double power) {
        this.feedPower = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(feedMotor.getPower()) > 1e-6) { // 1e-6 means 0.000001
            feedMotor.setPower(feedPower);
        }
    }

    public void start() {
        feedMotor.setPower(feedPower);
    }

    public void reverse() {
        feedMotor.setPower(-feedPower);
    }

    public void stop() {
        feedMotor.setPower(0.0);
    }

    public boolean isRunning() {
        return Math.abs(feedMotor.getPower()) > 1e-6;
    }
}
