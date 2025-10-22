package pedroPathing.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterSubsystem {

    // ===== Dashboard Tunables (same as ShooterTuner) =====
    public static double targetRPM = 2500;

    public static double kP = 22.0;
    public static double kI = 0.0;
    public static double kD = 8.0;
    public static double kF = 11.5;

    private static final double TICKS_PER_REV = 28.0;

    // ===== Hardware =====
    private final DcMotorEx shooter;
    private final FtcDashboard dashboard;
    private final MultipleTelemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = telemetry;

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE); // change to REVERSE if needed
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
    }



    public void update() {
        // Update PIDF coefficients live
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        // Compute velocity targets
        double targetVelocity = (targetRPM * TICKS_PER_REV) / 60.0;
        shooter.setVelocity(targetVelocity);

        double actualVelocity = shooter.getVelocity();
        double actualRPM = (actualVelocity * 60.0) / TICKS_PER_REV;
        double rpmError = targetRPM - actualRPM;

        // Driver Hub + Dashboard telemetry
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", actualVelocity);
        telemetry.addData("RPM Error", rpmError);
        telemetry.update();

        // Dashboard graph packet
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", targetRPM);
        packet.put("Actual RPM", actualRPM);
        packet.put("Error", rpmError);
        dashboard.sendTelemetryPacket(packet);
    }

    public void stop() {
        shooter.setPower(0);
    }

    public double getRPM() {
        return (shooter.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRPM - getRPM()) <= tolerance;
    }
}
