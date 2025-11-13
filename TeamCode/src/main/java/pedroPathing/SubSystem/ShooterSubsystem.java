package pedroPathing.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// Annotation allows variables to be tuned via the FTC Dashboard
@Config
public class ShooterSubsystem {

    // *** Dashboard Tunable Parameters ***
    public static double targetRPM = 2500; // Desired rotations per minute

    public static double kP = 12.0; // Proportional gain
    public static double kI = 0.05; // Integral gain
    public static double kD = 8.0; // Derivative gain
    public static double kF = 14.6; // Feedforward gain

    private static final double TICKS_PER_REV = 28.0; // Motor encoder constant

    // *** Hardware and Telemetry References ***
    private final DcMotorEx shooter; // Physical motor object
    private final FtcDashboard dashboard; // FtcDashboard instance
    private final MultipleTelemetry telemetry; // Combined telemetry for Driver Hub and Dashboard

    // *** Constructor ***
    public ShooterSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        // Initialize hardware references
        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = telemetry;

        // Configure motor settings
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        // Apply initial PIDF coefficients
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
    }


    // *** Main Update Loop Method ***
    public void update() {
        // Update PIDF coefficients live from the Dashboard variables
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        // Calculate target velocity in ticks per second
        double targetVelocity = (targetRPM * TICKS_PER_REV) / 60.0;
        shooter.setVelocity(targetVelocity);

        // Calculate actual RPM and error for logging
        double actualVelocity = shooter.getVelocity();
        double actualRPM = (actualVelocity * 60.0) / TICKS_PER_REV;
        double rpmError = targetRPM - actualRPM;

        // *** Telemetry Logging (Driver Hub & Dashboard) ***
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("RPM Error", rpmError);
        telemetry.update(); // Push data to Driver Hub

        // *** Dashboard Graphing Packet ***
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", targetRPM);
        packet.put("Actual RPM", actualRPM);
        packet.put("Error", rpmError);
        dashboard.sendTelemetryPacket(packet); // Send packet to Dashboard UI
    }

    // *** Utility Methods ***

    // Call before shooting to re-enable velocity mode
    public void startShooter() {
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Call to stop shooter motor
    public void stopShooter() {
        shooter.setPower(0);
    }


    // Stops the motor
    public void stop() {
        shooter.setPower(0);
    }

    // Returns the current motor speed in Rotations Per Minute (RPM)
    public double getRPM() {
        return (shooter.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    // Checks if the current RPM is within tolerance of the target RPM
    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRPM - getRPM()) <= tolerance;
    }
}
