package pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystem {
    private final Servo pushServo;
    private Servo pushServo2;

    private double pushEngagedPos = 0.3;   // Servo 1 on (hold)
    private double pushRetractedPos = 0.6; // Servo 1 off (hold)
    private boolean pushActive = false;    // Tracks state for servo 1 (hold)

    // ✅ New vars for servo 2
    private double push2EngagedPos = 1.0; // Servo 2 on (push)
    private double push2RetractedPos = 0.0; // Servo 2 off (push)
    private boolean push2Active = false; // Tracks state for servo 2 (push)

    public ServoSubsystem(Servo pushServo) {
        this.pushServo = pushServo;
        retractPush(); // Initialize servo 1
    }

    // ✅ Add a method to set second servo after init
    public void setPushServo2(Servo pushServo2) {
        this.pushServo2 = pushServo2;
        retractPush2(); // Initialize second servo
    }

    // ======= Original Servo (pushServo) =======
    public void togglePush() {
        pushActive = !pushActive;
        if (pushActive) {
            pushServo.setPosition(pushEngagedPos);
        } else {
            pushServo.setPosition(pushRetractedPos);
        }
    }

    public void engagePush() {
        pushActive = true;
        pushServo.setPosition(pushEngagedPos);
    }

    public void retractPush() {
        pushActive = false;
        pushServo.setPosition(pushRetractedPos);
    }

    public boolean isPushActive() {
        return pushActive;
    }

    public void setPushPositions(double engaged, double retracted) {
        this.pushEngagedPos = engaged;
        this.pushRetractedPos = retracted;
    }

    // ======= New Servo 2 (pushServo2) =======
    public void togglePush2() {
        if (pushServo2 == null) return; // safety
        push2Active = !push2Active;
        if (push2Active) {
            pushServo2.setPosition(push2EngagedPos);
        } else {
            pushServo2.setPosition(push2RetractedPos);
        }
    }

    public void engagePush2() {
        if (pushServo2 == null) return;
        push2Active = true;
        pushServo2.setPosition(push2EngagedPos);
    }

    public void retractPush2() {
        if (pushServo2 == null) return;
        push2Active = false;
        pushServo2.setPosition(push2RetractedPos);
    }

    public boolean isPush2Active() {
        return push2Active;
    }

    public void setPush2Positions(double engaged, double retracted) {
        this.push2EngagedPos = engaged;
        this.push2RetractedPos = retracted;
    }
}
