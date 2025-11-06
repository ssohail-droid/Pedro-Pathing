package pedroPathing.SubSystem;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * ServoSubsystem - Controls Push and Hold servos for artifact shooting
 *
 * SERVO ROLES:
 * - Push Servo: Pushes artifacts into the shooter mechanism
 * - Hold Servo: Holds artifacts in position (backup/timeout recovery)
 */
public class ServoSubsystem {
    private final Servo pushServo;
    private Servo holdServo;

    // Push Servo Positions
    private double pushEngagedPos = 0.9;   // Push servo engaged (pushing artifact)
    private double pushRetractedPos = 0.3; // Push servo retracted (ready position)
    private boolean pushActive = false;    // Tracks push servo state

    // Hold Servo Positions
    private double holdEngagedPos = 0.3;   // Hold servo engaged (holding artifact)
    private double holdRetractedPos = 0.6; // Hold servo retracted (released)
    private boolean holdActive = false;    // Tracks hold servo state

    /**
     * Constructor - Initialize with push servo
     * @param pushServo The primary push servo
     */
    public ServoSubsystem(Servo pushServo) {
        this.pushServo = pushServo;
        retractPush(); // Initialize push servo to retracted position
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HOLD SERVO SETUP (call this in init after creating ServoSubsystem)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set the hold servo (call this after construction)
     * @param holdServo The secondary hold servo
     */
    public void setHoldServo(Servo holdServo) {
        this.holdServo = holdServo;
        // Do NOT move servo automatically — leave it wherever it currently is
    }


    // ═══════════════════════════════════════════════════════════════════════
    // PUSH SERVO CONTROLS (Primary artifact pushing)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Toggle push servo between engaged and retracted
     */
    public void togglePush() {
        pushActive = !pushActive;
        if (pushActive) {
            pushServo.setPosition(pushEngagedPos);
        } else {
            pushServo.setPosition(pushRetractedPos);
        }
    }

    /**
     * Engage push servo (push artifact into shooter)
     */
    public void engagePush() {
        pushActive = true;
        pushServo.setPosition(pushEngagedPos);
    }

    /**
     * Retract push servo (return to ready position)
     */
    public void retractPush() {
        pushActive = false;
        pushServo.setPosition(pushRetractedPos);
    }

    /**
     * Check if push servo is currently active
     * @return true if push servo is engaged
     */
    public boolean isPushActive() {
        return pushActive;
    }

    /**
     * Set custom positions for push servo
     * @param engaged Position when pushing (0.0-1.0)
     * @param retracted Position when retracted (0.0-1.0)
     */
    public void setPushPositions(double engaged, double retracted) {
        this.pushEngagedPos = engaged;
        this.pushRetractedPos = retracted;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HOLD SERVO CONTROLS (Backup/timeout recovery)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Toggle hold servo between engaged and retracted
     */
    public void toggleHold() {
        if (holdServo == null) return; // Safety check
        holdActive = !holdActive;
        if (holdActive) {
            holdServo.setPosition(holdEngagedPos);
        } else {
            holdServo.setPosition(holdRetractedPos);
        }
    }

    /**
     * Engage hold servo (hold artifact in position)
     */
    public void engageHold() {
        if (holdServo == null) return; // Safety check
        holdActive = true;
        holdServo.setPosition(holdEngagedPos);
    }

    /**
     * Retract hold servo (release artifact)
     */
    public void retractHold() {
        if (holdServo == null) return; // Safety check
        holdActive = false;
        holdServo.setPosition(holdRetractedPos);
    }

    /**
     * Check if hold servo is currently active
     * @return true if hold servo is engaged
     */
    public boolean isHoldActive() {
        return holdActive;
    }

    /**
     * Set custom positions for hold servo
     * @param engaged Position when holding (0.0-1.0)
     * @param retracted Position when released (0.0-1.0)
     */
    public void setHoldPositions(double engaged, double retracted) {
        this.holdEngagedPos = engaged;
        this.holdRetractedPos = retracted;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get current push servo position
     * @return Current position (0.0-1.0)
     */
    public double getPushPosition() {
        return pushServo.getPosition();
    }

    /**
     * Get current hold servo position
     * @return Current position (0.0-1.0), or -1 if hold servo not set
     */
    public double getHoldPosition() {
        if (holdServo == null) return -1;
        return holdServo.getPosition();
    }

    /**
     * Emergency stop - retract both servos
     */
    public void emergencyStop() {
        retractPush();
        retractHold();
    }
}