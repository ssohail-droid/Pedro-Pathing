package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
//@TeleOp(name = "DriveOnly blue end", group = "Refactored")
public class DriveTrain extends OpMode {

    private Follower follower;
    private DcMotor intake = null;

    // Keep start and target pose for potential auto-pathing use
    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose targetPose = new Pose(19.25, 20.85, Math.toRadians(312));

    // --- RENAMED TOGGLE VARIABLES ---
    private boolean leftBumperPressed = false; // Tracks state for the new toggle button
    private boolean yPressed = false; // Tracks state for the auto-path start button
    // --------------------------------

    private boolean navigating = false;
    private boolean arrived = false;

    private boolean intakeOn = false;
    private final double INTAKE_POWER = 0.5; // Set your desired power level

    // Keep alignment constants, though alignment sensor logic is removed.
    // Alignment is now purely kinematic (position/heading check).
    public static double POSITION_TOLERANCE = 2.0;
    public static double HEADING_TOLERANCE_DEG = 5;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // ADDED INTAKE INITIALIZATION
        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.REVERSE); // Adjust direction as needed
            intake.setPower(0);
        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find 'intake' motor in hardware map!");
        }

        telemetry.addLine("Initialized. Drive and Odometry Only.");
        telemetry.addLine("Press LEFT BUMPER to TOGGLE INTAKE.");
        telemetry.addLine("Press Y to start auto-path.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // === Gamepad 1: INTAKE TOGGLE (Left Bumper) ===
        if (gamepad1.left_bumper && !leftBumperPressed) {
            leftBumperPressed = true;
            // Toggle the intake state
            intakeOn = !intakeOn;
            telemetry.addData("Intake Toggled", intakeOn ? "ON" : "OFF");
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }

        // === Gamepad 1: START AUTO NAVIGATION (Y Button) ===
        if (gamepad1.y && !yPressed && !navigating) {
            yPressed = true;
            startAutoNavigation();
        } else if (!gamepad1.y) {
            yPressed = false;
        }

        // === Gamepad 1: CANCEL NAVIGATION (Right Bumper) ===
        // The right bumper is now the sole cancel button
        if (gamepad1.right_bumper && navigating) {
            navigating = false;
            arrived = false;
            follower.breakFollowing();
            follower.startTeleopDrive();
            gamepad1.rumble(300);
            telemetry.addLine("Navigation canceled manually (Right Bumper).");
        }

        // === AUTO NAVIGATION ===
        if (navigating && !arrived) {
            follower.update();

            if (hasArrivedAtTarget()) {
                navigating = false;
                arrived = true;

                follower.breakFollowing();
                follower.setPose(targetPose);
                follower.startTeleopDrive(); // Switch back to TeleOp mode
            }
        }

        // === INTAKE CONTROL (Based on toggle state) ===
        if (intake != null) {
            intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
        }
        // ----------------------------------------------

        // === DEFAULT DRIVE (Main Control for Drive Motors) ===
        if (!navigating) {
            follower.setTeleOpMovementVectors(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // === Telemetry ===
        telemetry.addLine(navigating ? "Navigating to target..." : "Manual drive mode");
        telemetry.addData("Intake Status (LB)", intakeOn ? "ON" : "OFF");

        telemetry.addLine("=== Pose (Odometry) ===");
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading°", "%.1f", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    // === Navigation Helper Methods (Unchanged) ===

    private void startAutoNavigation() {
        navigating = true;
        arrived = false;

        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(follower.getPose()),
                                new Point(targetPose)
                        ))
                        .setLinearHeadingInterpolation(
                                follower.getPose().getHeading(),
                                targetPose.getHeading()
                        )
                        .build()
        );
        telemetry.addLine("Auto-Path Started (Y button)");
    }

    private boolean hasArrivedAtTarget() {
        Pose current = follower.getPose();
        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distanceError = Math.hypot(dx, dy);
        double headingError = normalizeAngle(targetPose.getHeading() - current.getHeading());

        return distanceError < POSITION_TOLERANCE &&
                Math.abs(headingError) < Math.toRadians(HEADING_TOLERANCE_DEG);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}