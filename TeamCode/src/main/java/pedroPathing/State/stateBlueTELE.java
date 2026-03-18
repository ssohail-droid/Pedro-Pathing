package pedroPathing.State;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Blue TELE", group = "Main")
@Config
public class stateBlueTELE extends OpMode {

    // ──────────────── PEDRO PATHING ────────────────
    private Follower follower;
    private final Pose startPose = new Pose(49.357495881383855, 123.7693574958814, Math.toRadians(180));

    public static Pose TARGET_A = new Pose(44.94409937888198, 99.05590062111803, Math.toRadians(134));
    public static Pose TARGET_B = new Pose(57.91304347826086, 124.09937888198759, Math.toRadians(167));
    public static Pose TARGET_C = new Pose(57.018633540372676, 86.75776397515531, Math.toRadians(134));

    public static double POS_TOL          = 2.0;
    public static double HEAD_TOL_DEG     = 0.01;
    public static double SLOW_MULT        = 0.35;
    public static double TRIGGER_MIN_MULT = 0.25;

    private Pose activeTarget      = null;
    private boolean navigating     = false;
    private boolean slowMode       = false;

    private boolean lastSlowToggle = false;
    private boolean lastXDrive     = false;
    private boolean lastYDrive     = false;
    private boolean lastBDrive     = false;

    // ──────────────── INTAKE / TRANSPORT / SHOOTER ────────────────
    private DcMotorEx upPipe;
    private DcMotorEx midPipe;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private Servo gateServo;
    private Servo hoodServo;

    private int shooterToggle       = 0; // 0=off, 1=RPM_A, 2=RPM_B
    private boolean lastShooterBtnA = false;
    private boolean lastShooterBtnB = false;

    private static final double TICKS_PER_REV = 28.0;

    public static double SHOOT_RPM_A  = 2530;
    public static double SHOOT_RPM_B  = 3200;
    public static double HOOD_POS_A   = 0.675;
    public static double HOOD_POS_B   = 0.80;
    public static double SHOOTER_P    = 70;
    public static double SHOOTER_I    = 0;
    public static double SHOOTER_D    = 8;
    public static double SHOOTER_F    = 13.5;
    public static double upPipePower  = 1.0;
    public static double midPipePower = 1.0;
    public static double intakePower  = 1.0;
    private boolean rb2Pressed = false;
    private boolean sequenceActive = false;
    private long sequenceStartTime = 0;
    private static final long GATE_DELAY_MS = 300; // adjust as needed

    // ──────────────────────────────────────────────
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pedro Pathing
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        // Transport hardware
        upPipe    = hardwareMap.get(DcMotorEx.class, "upPipe");
        midPipe   = hardwareMap.get(DcMotorEx.class, "midPipe");
        intake    = hardwareMap.get(DcMotorEx.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "stop");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        upPipe.setDirection(DcMotorSimple.Direction.REVERSE);
        midPipe.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        gateServo.setPosition(0.4);
        hoodServo.setPosition(HOOD_POS_A);

        // Shooter hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F)
        );
    }

    @Override
    public void loop() {

        // ═══════════════════════════════════════════
        //  GAMEPAD 1 — DRIVE
        // ═══════════════════════════════════════════

        /* --- Slow mode toggle (A) --- */
        boolean slowTogglePressed = gamepad1.a && !lastSlowToggle;
        if (slowTogglePressed) slowMode = !slowMode;
        lastSlowToggle = gamepad1.a;

        /* --- Autonomous target select (X / Y / B) --- */
        boolean xPressed = gamepad1.x && !lastXDrive;
        boolean yPressed = gamepad1.y && !lastYDrive;
        boolean bPressed = gamepad1.b && !lastBDrive;

        if (!navigating) {
            if (xPressed) {
                activeTarget = TARGET_A;
                navigating   = true;
            } else if (yPressed) {
                activeTarget = TARGET_B;
                navigating   = true;
            } else if (bPressed) {
                activeTarget = TARGET_C;
                navigating   = true;
            }

            if (navigating) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(activeTarget)
                                ))
                                .setLinearHeadingInterpolation(
                                        follower.getPose().getHeading(),
                                        activeTarget.getHeading()
                                )
                                .build()
                );
            }
        }

        lastXDrive = gamepad1.x;
        lastYDrive = gamepad1.y;
        lastBDrive = gamepad1.b;

        /* --- Cancel navigation (bumpers) --- */
        if ((gamepad1.left_bumper || gamepad1.right_bumper) && navigating) {
            cancelNavigation();
        }

        /* --- Drive / follower update --- */
        if (navigating) {
            follower.update();
            if (arrived()) cancelNavigation();
        } else {
            double trigger     = gamepad1.right_trigger;
            double triggerMult = 1.0 - trigger * (1.0 - TRIGGER_MIN_MULT);
            double finalMult   = slowMode ? Math.min(SLOW_MULT, triggerMult) : triggerMult;

            follower.setTeleOpMovementVectors(
                    gamepad1.left_stick_y * finalMult,
                    gamepad1.left_stick_x * finalMult,
                    -gamepad1.right_stick_x * finalMult,
                    false
            );
            follower.update();
        }

        // ═══════════════════════════════════════════
        //  GAMEPAD 2 — INTAKE / TRANSPORT / SHOOTER
        // ═══════════════════════════════════════════

        /* --- Shooter toggle A (Y) --- SHOOT_RPM_A --- */
        boolean shooterBtnA = gamepad2.y;
        if (shooterBtnA && !lastShooterBtnA) {
            if (shooterToggle == 1) {
                shooterToggle = 0;
            } else {
                shooterToggle = 1;
                hoodServo.setPosition(HOOD_POS_A);
            }


        }
        lastShooterBtnA = shooterBtnA;

        /* --- Shooter toggle B (B) --- SHOOT_RPM_B --- */
        boolean shooterBtnB = gamepad2.b;
        if (shooterBtnB && !lastShooterBtnB) {
            if (shooterToggle == 2) {
                shooterToggle = 0;
            } else {
                shooterToggle = 2;
                hoodServo.setPosition(HOOD_POS_B);
            }
        }
        lastShooterBtnB = shooterBtnB;

        /* --- Shooter velocity --- */
//        if (shooterToggle == 1) {
//            shooter.setVelocity((SHOOT_RPM_A * TICKS_PER_REV) / 60.0);
//        } else if (shooterToggle == 2) {
//            shooter.setVelocity((SHOOT_RPM_B * TICKS_PER_REV) / 60.0);
//        } else {
//            shooter.setVelocity(0);
//        }

        /* --- Shooter velocity --- */
        if (shooterToggle == 1) {
            shooter.setVelocity((SHOOT_RPM_A * TICKS_PER_REV) / 60.0);
        } else if (shooterToggle == 2) {
            shooter.setVelocity((SHOOT_RPM_B * TICKS_PER_REV) / 60.0);
        } else {
            shooter.setVelocity(0);
        }

        /* --- Rumble while shooter is running --- */
        if (shooterToggle != 0) {
            gamepad2.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else {
            gamepad2.stopRumble();
        }

        /* --- Preset: midPipe + intake (left bumper) --- */
        if (gamepad2.left_bumper) {
            midPipe.setPower(0.3);
            intake.setPower(intakePower);
            gateServo.setPosition(0.1);
            hoodServo.setPosition(HOOD_POS_A);
        }

        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !rb2Pressed) {
// Rising edge detected - start sequence
            gateServo.setPosition(0.4);
            sequenceActive = true;
            sequenceStartTime = System.currentTimeMillis();
        }
        rb2Pressed = rb2;

        if (sequenceActive && (System.currentTimeMillis() - sequenceStartTime >= GATE_DELAY_MS)) {
            upPipe.setPower(0.5);
            midPipe.setPower(midPipePower);
            intake.setPower(intakePower);
            hoodServo.setPosition(HOOD_POS_A);
            sequenceActive = false;
        }


        /* --- Preset: all transport motors (right bumper) --- */
//        if (gamepad2.right_bumper) {
//            upPipe.setPower(0.5);
//            midPipe.setPower(midPipePower);
//            intake.setPower(intakePower);
//            gateServo.setPosition(0.4);
//            hoodServo.setPosition(0.75);
//        }
//        if (gamepad2.a) {
//            gateServo.setPosition(0.4);
//        }
        if (gamepad2.share) {
            upPipe.setPower(-1);
            midPipe.setPower(-1);
            intake.setPower(-1);
        }

        /* --- Stop all transport motors (X) --- */
        if (gamepad2.x) {
            upPipe.setPower(0);
            midPipe.setPower(0);
            intake.setPower(0);
        }

        // ═══════════════════════════════════════════
        //  TELEMETRY
        // ═══════════════════════════════════════════
        telemetry.addData("── DRIVE ──────────────", "");
        telemetry.addData("Slow Mode",   slowMode   ? "ON" : "OFF");
        telemetry.addData("Navigating",  navigating);

        telemetry.addData("── SHOOTER ────────────", "");
        telemetry.addData("Shooter",     shooterToggle == 0 ? "OFF" : "RPM_" + (shooterToggle == 1 ? "A" : "B"));
        telemetry.addData("Target RPM",  shooterToggle == 1 ? SHOOT_RPM_A : shooterToggle == 2 ? SHOOT_RPM_B : 0);
        telemetry.addData("Target Hood", shooterToggle == 1 ? HOOD_POS_A  : shooterToggle == 2 ? HOOD_POS_B  : 0);
        telemetry.addData("Current RPM", shooter.getVelocity() * 60.0 / TICKS_PER_REV);

        telemetry.addData("── TRANSPORT ──────────", "");
        telemetry.addData("upPipe Power",  upPipe.getPower());
        telemetry.addData("midPipe Power", midPipe.getPower());
        telemetry.addData("intake Power",  intake.getPower());

        telemetry.update();
    }

    // ──────────────────────────────────────────────
    //  HELPERS
    // ──────────────────────────────────────────────
    private void cancelNavigation() {
        navigating   = false;
        activeTarget = null;
        follower.breakFollowing();
        follower.startTeleopDrive();
    }

    private boolean arrived() {
        if (activeTarget == null) return false;

        Pose p    = follower.getPose();
        double dx = activeTarget.getX() - p.getX();
        double dy = activeTarget.getY() - p.getY();
        double dist = Math.hypot(dx, dy);

        double headingErr = Math.toDegrees(
                Math.atan2(
                        Math.sin(p.getHeading() - activeTarget.getHeading()),
                        Math.cos(p.getHeading() - activeTarget.getHeading())
                )
        );

        return dist < POS_TOL && Math.abs(headingErr) < HEAD_TOL_DEG;
    }
}