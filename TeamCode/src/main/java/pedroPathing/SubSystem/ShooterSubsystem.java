package pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ShooterSubsystem {
    private final DcMotor shooterMotor1, shooterMotor2;
    private double shooterPower = 0.48; // configurable
    private double idlePower = 0.0;    // configurable small hold

    public ShooterSubsystem(DcMotor shooterMotor1, DcMotor shooterMotor2) {
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;

        this.shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setIdle(); // default to idle on init
    }

    public void setShooterPower(double power) {
        this.shooterPower = Math.max(0.0, Math.min(1.0, power));
        if (isSpinning()) {
            applySpinPower();
        }
    }

    public void setIdlePower(double power) {
        this.idlePower = Math.max(0.0, Math.min(0.25, power));
        if (!isSpinning()) {
            applyIdlePower();
        }
    }

    public void spinUp() {
        applySpinPower();
    }

    public void setIdle() {
        applyIdlePower();
    }

    public void stop() {
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }

    public boolean isSpinning() {
        return Math.abs(shooterMotor1.getPower()) > idlePower + 1e-6 ||
                Math.abs(shooterMotor2.getPower()) > idlePower + 1e-6;
    }

    private void applySpinPower() {
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1.setPower(-shooterPower);
        shooterMotor2.setPower( shooterPower);
    }

    private void applyIdlePower() {
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setPower(-idlePower);
        shooterMotor2.setPower( idlePower);
    }
}