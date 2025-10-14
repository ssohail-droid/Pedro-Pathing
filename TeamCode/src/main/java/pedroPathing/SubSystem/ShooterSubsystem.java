package pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterSubsystem {
    private final DcMotorEx shooterMotor1, shooterMotor2;
    private double targetVelocity = 1800; // ticks per second (example)
    private double idleVelocity = 200;    // ticks per second idle spin

    public ShooterSubsystem(DcMotorEx shooterMotor1, DcMotorEx shooterMotor2) {
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setIdle();
    }

    public void setTargetVelocity(double velocityTicksPerSec) {
        this.targetVelocity = Math.max(0.0, velocityTicksPerSec);
        if (isSpinning()) applySpinVelocity();
    }

    public void setIdleVelocity(double velocityTicksPerSec) {
        this.idleVelocity = Math.max(0.0, velocityTicksPerSec);
        if (!isSpinning()) applyIdleVelocity();
    }

    public void spinUp() {
        applySpinVelocity();
    }

    public void setIdle() {
        applyIdleVelocity();
    }

    public void stop() {
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }

    public boolean isSpinning() {
        // Uses velocity feedback to detect spin state
        return Math.abs(shooterMotor1.getVelocity()) > idleVelocity + 10 ||
                Math.abs(shooterMotor2.getVelocity()) > idleVelocity + 10;
    }

    private void applySpinVelocity() {
        shooterMotor1.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);
    }

    private void applyIdleVelocity() {
        shooterMotor1.setVelocity(-idleVelocity);
        shooterMotor2.setVelocity(-idleVelocity);
    }

    public double getShooterVelocity() {
        return (Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity())) / 2.0;
    }
}
