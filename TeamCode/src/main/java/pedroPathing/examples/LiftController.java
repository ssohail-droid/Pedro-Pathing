package pedroPathing.examples;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftController {
    private final DcMotor liftMotor;
    private final int bottomLimit;
    private final int topLimit;
    private final int speed;
    private int holdPosition = 0;

    public LiftController(DcMotor liftMotor, int bottomLimit, int topLimit, int speed) {
        this.liftMotor = liftMotor;
        this.bottomLimit = bottomLimit;
        this.topLimit = topLimit;
        this.speed = speed;

        // Reverse motor direction if needed
        liftMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse the motor direction

        // Initial configuration
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Move lift up if we haven't reached the top limit */
    public void moveUp() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (liftMotor.getCurrentPosition() < topLimit) {
            liftMotor.setPower(speed / 100.0);
        } else {
            liftMotor.setPower(0);
        }
    }

    /** Move lift down if we haven't reached the bottom limit */
    public void moveDown() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (liftMotor.getCurrentPosition() > bottomLimit) {
            liftMotor.setPower(-speed / 100.0);
        } else {
            liftMotor.setPower(0);
        }
    }

    /** Stop motor and hold position */
    public void stop() {
        holdPosition = liftMotor.getCurrentPosition();

        liftMotor.setTargetPosition(holdPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower((speed * 0.25) / 100.0); // Apply holding power
    }

    /** Read-only: what encoder count we're at right now */
    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }
}
