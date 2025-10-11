package pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystem {
    private final Servo pushServo;
    private double pushEngagedPos = 1.0;   // Servo on
    private double pushRetractedPos = 0.0;  // Servo off
    private boolean pushActive = false;     // Tracks state

    public ServoSubsystem(Servo pushServo) {
        this.pushServo = pushServo;
        retractPush(); // Initialize to off position
    }

    // Toggle between on/off
    public void togglePush() {
        pushActive = !pushActive;
        if (pushActive) {
            pushServo.setPosition(pushEngagedPos);
        } else {
            pushServo.setPosition(pushRetractedPos);
        }
    }

    // Explicit on/off commands (optional)
    public void engagePush() {
        pushActive = true;
        pushServo.setPosition(pushEngagedPos);
    }

    public void retractPush() {
        pushActive = false;
        pushServo.setPosition(pushRetractedPos);
    }

    // For telemetry/debug
    public boolean isPushActive() {
        return pushActive;
    }

    // Optional tuning at runtime
    public void setPushPositions(double engaged, double retracted) {
        this.pushEngagedPos = engaged;
        this.pushRetractedPos = retracted;
    }
}
