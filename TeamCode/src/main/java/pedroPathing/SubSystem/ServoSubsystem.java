package pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystem {
    private final Servo holdServo, pushServo;

    // Expose positions so OpModes can choose behavior
    private double holdEngagedPos = 1.0;
    private double holdReleasedPos = 0.0;
    private double pushEngagedPos = 1.0;
    private double pushRetractPos = 0.0;

    public ServoSubsystem(Servo holdServo, Servo pushServo) {
        this.holdServo = holdServo;
        this.pushServo = pushServo;
        // Initialize to safe state
        engageHold();
        retractPush();
    }

    // Hold control
    public void engageHold() { holdServo.setPosition(holdEngagedPos); }
    public void releaseHold() { holdServo.setPosition(holdReleasedPos); }

    // Push control
    public void engagePush() { pushServo.setPosition(pushEngagedPos); }
    public void retractPush() { pushServo.setPosition(pushRetractPos); }

    // Optional setters to tune positions at runtime
    public void setHoldPositions(double engaged, double released) {
        this.holdEngagedPos = engaged;
        this.holdReleasedPos = released;
    }

    public void setPushPositions(double engaged, double retracted) {
        this.pushEngagedPos = engaged;
        this.pushRetractPos = retracted;
    }
}
