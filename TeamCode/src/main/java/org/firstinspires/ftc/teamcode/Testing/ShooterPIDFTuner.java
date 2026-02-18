package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
//@TeleOp(name = "Shooter PIDF Solo Tuner", group = "Tuning")
public class ShooterPIDFTuner extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private FtcDashboard dashboard;

    // ======= 📊 DASHBOARD TUNABLE VALUES =======
    public static double TARGET_RPM = 2000;
    public static double TICKS_PER_REV = 28.0;

    // PIDF Constants (Adjust these in Dashboard)
    public static double P = 65;
    public static double I = 2.0;
    public static double D = 0.00009;
    public static double F = 13.0;

    private boolean shooterOn = false;
    private boolean lastButtonA = false;

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize Dashboard Telemetry
        dashboard = FtcDashboard.getInstance();
        // This line sends telemetry to BOTH the DS and the Dashboard Graph
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready! Use Gamepad 1 - A to Toggle");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle Logic
            if (gamepad1.a && !lastButtonA) {
                shooterOn = !shooterOn;
            }
            lastButtonA = gamepad1.a;

            // Apply PIDF from Dashboard
            shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);

            // Calculate Target Velocity
            double targetTPS = shooterOn ? (TARGET_RPM * TICKS_PER_REV) / 60.0 : 0.0;
            shooterMotor.setVelocity(targetTPS);

            // Calculate Current RPM for easy reading
            double currentTPS = shooterMotor.getVelocity();
            double currentRPM = (currentTPS * 60.0) / TICKS_PER_REV;
            double error = TARGET_RPM - currentRPM;

            // ======= 📈 DASHBOARD TELEMETRY =======
            // We use specific keys so they appear as separate lines on the graph
            telemetry.addData("Target RPM", shooterOn ? TARGET_RPM : 0);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Error RPM", shooterOn ? error : 0);
            telemetry.addData("Status", shooterOn ? "RUNNING" : "STOPPED");
            telemetry.update();
        }
    }
}