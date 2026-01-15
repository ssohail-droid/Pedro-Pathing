package pedroPathing.Jan10.TELEOP.DASAUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Blue Auto (Patterns + CR Shoot)", group = "Main")
public class BlueAutoPatterns extends OpMode {

    /* =========================================================
       =================== YOU TUNE HERE =======================
       ========================================================= */

    /** Manual pattern select for now (camera will set this later) */
    public static int SELECTED_PATTERN = 0; // 0, 1, 2

    /** Pattern shot orders (per phase) */
    // Pattern 0: Safe baseline
    private static final String P0_PRELOAD = "ABC";
    private static final String P0_ROW1    = "ABC";
    private static final String P0_ROW2    = "ACB";

    // Pattern 1: Aggressive / center-first example
    private static final String P1_PRELOAD = "CBA";
    private static final String P1_ROW1    = "CBA";
    private static final String P1_ROW2    = "BAC";

    // Pattern 2: Jam-avoid example (repeats allowed)
    private static final String P2_PRELOAD = "BCA";
    private static final String P2_ROW1    = "BCA";
    private static final String P2_ROW2    = "ABC";

    /** Spin positions for A/B/C (same across phases for now; change if you want) */
    public static double SHOOT_A = 0.56;
    public static double SHOOT_B = 0.28;
    public static double SHOOT_C = 0.00;

    /** Stage enum for CR control during shooting */
    private enum Stage { MOVE, SETTLE, KICK, POST }

    /**
     * FULL CR CONTROL:
     * For EACH phase (PRELOAD/ROW1/ROW2), EACH letter (A/B/C), EACH stage (MOVE/SETTLE/KICK/POST),
     * define left and right CR powers.
     *
     * Start simple: make them the same as your old shoot feed, then tune.
     */
    // Default “old behavior”: ON through most of shot
    public static double OLD_CR_L = 1.0;
    public static double OLD_CR_R = -1.0;

    // Pattern 0 (Safe): Example defaults (you will tune these!)
    // PRELOAD
    public static double P0_PRE_A_MOVE_L = 0.0,  P0_PRE_A_MOVE_R = 0.0;
    public static double P0_PRE_A_SET_L  = OLD_CR_L, P0_PRE_A_SET_R  = OLD_CR_R;
    public static double P0_PRE_A_KICK_L = OLD_CR_L, P0_PRE_A_KICK_R = OLD_CR_R;
    public static double P0_PRE_A_POST_L = OLD_CR_L, P0_PRE_A_POST_R = OLD_CR_R;

    public static double P0_PRE_B_MOVE_L = 0.0,  P0_PRE_B_MOVE_R = 0.0;
    public static double P0_PRE_B_SET_L  = OLD_CR_L, P0_PRE_B_SET_R  = OLD_CR_R;
    public static double P0_PRE_B_KICK_L = OLD_CR_L, P0_PRE_B_KICK_R = OLD_CR_R;
    public static double P0_PRE_B_POST_L = OLD_CR_L, P0_PRE_B_POST_R = OLD_CR_R;

    public static double P0_PRE_C_MOVE_L = 0.0,  P0_PRE_C_MOVE_R = 0.0;
    public static double P0_PRE_C_SET_L  = OLD_CR_L, P0_PRE_C_SET_R  = OLD_CR_R;
    public static double P0_PRE_C_KICK_L = OLD_CR_L, P0_PRE_C_KICK_R = OLD_CR_R;
    public static double P0_PRE_C_POST_L = OLD_CR_L, P0_PRE_C_POST_R = OLD_CR_R;

    // ROW 1
    public static double P0_R1_A_MOVE_L = 0.0,  P0_R1_A_MOVE_R = 0.0;
    public static double P0_R1_A_SET_L  = OLD_CR_L, P0_R1_A_SET_R  = OLD_CR_R;
    public static double P0_R1_A_KICK_L = OLD_CR_L, P0_R1_A_KICK_R = OLD_CR_R;
    public static double P0_R1_A_POST_L = OLD_CR_L, P0_R1_A_POST_R = OLD_CR_R;

    public static double P0_R1_B_MOVE_L = 0.0,  P0_R1_B_MOVE_R = 0.0;
    public static double P0_R1_B_SET_L  = OLD_CR_L, P0_R1_B_SET_R  = OLD_CR_R;
    public static double P0_R1_B_KICK_L = OLD_CR_L, P0_R1_B_KICK_R = OLD_CR_R;
    public static double P0_R1_B_POST_L = OLD_CR_L, P0_R1_B_POST_R = OLD_CR_R;

    public static double P0_R1_C_MOVE_L = 0.0,  P0_R1_C_MOVE_R = 0.0;
    public static double P0_R1_C_SET_L  = OLD_CR_L, P0_R1_C_SET_R  = OLD_CR_R;
    public static double P0_R1_C_KICK_L = OLD_CR_L, P0_R1_C_KICK_R = OLD_CR_R;
    public static double P0_R1_C_POST_L = OLD_CR_L, P0_R1_C_POST_R = OLD_CR_R;

    // ROW 2
    public static double P0_R2_A_MOVE_L = 0.0,  P0_R2_A_MOVE_R = 0.0;
    public static double P0_R2_A_SET_L  = OLD_CR_L, P0_R2_A_SET_R  = OLD_CR_R;
    public static double P0_R2_A_KICK_L = OLD_CR_L, P0_R2_A_KICK_R = OLD_CR_R;
    public static double P0_R2_A_POST_L = OLD_CR_L, P0_R2_A_POST_R = OLD_CR_R;

    public static double P0_R2_B_MOVE_L = 0.0,  P0_R2_B_MOVE_R = 0.0;
    public static double P0_R2_B_SET_L  = OLD_CR_L, P0_R2_B_SET_R  = OLD_CR_R;
    public static double P0_R2_B_KICK_L = OLD_CR_L, P0_R2_B_KICK_R = OLD_CR_R;
    public static double P0_R2_B_POST_L = OLD_CR_L, P0_R2_B_POST_R = OLD_CR_R;

    public static double P0_R2_C_MOVE_L = 0.0,  P0_R2_C_MOVE_R = 0.0;
    public static double P0_R2_C_SET_L  = OLD_CR_L, P0_R2_C_SET_R  = OLD_CR_R;
    public static double P0_R2_C_KICK_L = OLD_CR_L, P0_R2_C_KICK_R = OLD_CR_R;
    public static double P0_R2_C_POST_L = OLD_CR_L, P0_R2_C_POST_R = OLD_CR_R;

    /**
     * Patterns 1 & 2: start identical to pattern 0 (safe).
     * Once this runs, you’ll tune these variables as needed.
     */
    // (Pattern 1 mirrors Pattern 0 initially)
    public static double P1_PRE_A_MOVE_L = 0.0, P1_PRE_A_MOVE_R = 0.0;
    public static double P1_PRE_A_SET_L  = OLD_CR_L, P1_PRE_A_SET_R = OLD_CR_R;
    public static double P1_PRE_A_KICK_L = OLD_CR_L, P1_PRE_A_KICK_R = OLD_CR_R;
    public static double P1_PRE_A_POST_L = OLD_CR_L, P1_PRE_A_POST_R = OLD_CR_R;

    public static double P1_PRE_B_MOVE_L = 0.0, P1_PRE_B_MOVE_R = 0.0;
    public static double P1_PRE_B_SET_L  = OLD_CR_L, P1_PRE_B_SET_R = OLD_CR_R;
    public static double P1_PRE_B_KICK_L = OLD_CR_L, P1_PRE_B_KICK_R = OLD_CR_R;
    public static double P1_PRE_B_POST_L = OLD_CR_L, P1_PRE_B_POST_R = OLD_CR_R;

    public static double P1_PRE_C_MOVE_L = 0.0, P1_PRE_C_MOVE_R = 0.0;
    public static double P1_PRE_C_SET_L  = OLD_CR_L, P1_PRE_C_SET_R = OLD_CR_R;
    public static double P1_PRE_C_KICK_L = OLD_CR_L, P1_PRE_C_KICK_R = OLD_CR_R;
    public static double P1_PRE_C_POST_L = OLD_CR_L, P1_PRE_C_POST_R = OLD_CR_R;

    public static double P1_R1_A_MOVE_L = 0.0, P1_R1_A_MOVE_R = 0.0;
    public static double P1_R1_A_SET_L  = OLD_CR_L, P1_R1_A_SET_R = OLD_CR_R;
    public static double P1_R1_A_KICK_L = OLD_CR_L, P1_R1_A_KICK_R = OLD_CR_R;
    public static double P1_R1_A_POST_L = OLD_CR_L, P1_R1_A_POST_R = OLD_CR_R;

    public static double P1_R1_B_MOVE_L = 0.0, P1_R1_B_MOVE_R = 0.0;
    public static double P1_R1_B_SET_L  = OLD_CR_L, P1_R1_B_SET_R = OLD_CR_R;
    public static double P1_R1_B_KICK_L = OLD_CR_L, P1_R1_B_KICK_R = OLD_CR_R;
    public static double P1_R1_B_POST_L = OLD_CR_L, P1_R1_B_POST_R = OLD_CR_R;

    public static double P1_R1_C_MOVE_L = 0.0, P1_R1_C_MOVE_R = 0.0;
    public static double P1_R1_C_SET_L  = OLD_CR_L, P1_R1_C_SET_R = OLD_CR_R;
    public static double P1_R1_C_KICK_L = OLD_CR_L, P1_R1_C_KICK_R = OLD_CR_R;
    public static double P1_R1_C_POST_L = OLD_CR_L, P1_R1_C_POST_R = OLD_CR_R;

    public static double P1_R2_A_MOVE_L = 0.0, P1_R2_A_MOVE_R = 0.0;
    public static double P1_R2_A_SET_L  = OLD_CR_L, P1_R2_A_SET_R = OLD_CR_R;
    public static double P1_R2_A_KICK_L = OLD_CR_L, P1_R2_A_KICK_R = OLD_CR_R;
    public static double P1_R2_A_POST_L = OLD_CR_L, P1_R2_A_POST_R = OLD_CR_R;

    public static double P1_R2_B_MOVE_L = 0.0, P1_R2_B_MOVE_R = 0.0;
    public static double P1_R2_B_SET_L  = OLD_CR_L, P1_R2_B_SET_R = OLD_CR_R;
    public static double P1_R2_B_KICK_L = OLD_CR_L, P1_R2_B_KICK_R = OLD_CR_R;
    public static double P1_R2_B_POST_L = OLD_CR_L, P1_R2_B_POST_R = OLD_CR_R;

    public static double P1_R2_C_MOVE_L = 0.0, P1_R2_C_MOVE_R = 0.0;
    public static double P1_R2_C_SET_L  = OLD_CR_L, P1_R2_C_SET_R = OLD_CR_R;
    public static double P1_R2_C_KICK_L = OLD_CR_L, P1_R2_C_KICK_R = OLD_CR_R;
    public static double P1_R2_C_POST_L = OLD_CR_L, P1_R2_C_POST_R = OLD_CR_R;

    // (Pattern 2 mirrors Pattern 0 initially)
    public static double P2_PRE_A_MOVE_L = 0.0, P2_PRE_A_MOVE_R = 0.0;
    public static double P2_PRE_A_SET_L  = OLD_CR_L, P2_PRE_A_SET_R = OLD_CR_R;
    public static double P2_PRE_A_KICK_L = OLD_CR_L, P2_PRE_A_KICK_R = OLD_CR_R;
    public static double P2_PRE_A_POST_L = OLD_CR_L, P2_PRE_A_POST_R = OLD_CR_R;

    public static double P2_PRE_B_MOVE_L = 0.0, P2_PRE_B_MOVE_R = 0.0;
    public static double P2_PRE_B_SET_L  = OLD_CR_L, P2_PRE_B_SET_R = OLD_CR_R;
    public static double P2_PRE_B_KICK_L = OLD_CR_L, P2_PRE_B_KICK_R = OLD_CR_R;
    public static double P2_PRE_B_POST_L = OLD_CR_L, P2_PRE_B_POST_R = OLD_CR_R;

    public static double P2_PRE_C_MOVE_L = 0.0, P2_PRE_C_MOVE_R = 0.0;
    public static double P2_PRE_C_SET_L  = OLD_CR_L, P2_PRE_C_SET_R = OLD_CR_R;
    public static double P2_PRE_C_KICK_L = OLD_CR_L, P2_PRE_C_KICK_R = OLD_CR_R;
    public static double P2_PRE_C_POST_L = OLD_CR_L, P2_PRE_C_POST_R = OLD_CR_R;

    public static double P2_R1_A_MOVE_L = 0.0, P2_R1_A_MOVE_R = 0.0;
    public static double P2_R1_A_SET_L  = OLD_CR_L, P2_R1_A_SET_R = OLD_CR_R;
    public static double P2_R1_A_KICK_L = OLD_CR_L, P2_R1_A_KICK_R = OLD_CR_R;
    public static double P2_R1_A_POST_L = OLD_CR_L, P2_R1_A_POST_R = OLD_CR_R;

    public static double P2_R1_B_MOVE_L = 0.0, P2_R1_B_MOVE_R = 0.0;
    public static double P2_R1_B_SET_L  = OLD_CR_L, P2_R1_B_SET_R = OLD_CR_R;
    public static double P2_R1_B_KICK_L = OLD_CR_L, P2_R1_B_KICK_R = OLD_CR_R;
    public static double P2_R1_B_POST_L = OLD_CR_L, P2_R1_B_POST_R = OLD_CR_R;

    public static double P2_R1_C_MOVE_L = 0.0, P2_R1_C_MOVE_R = 0.0;
    public static double P2_R1_C_SET_L  = OLD_CR_L, P2_R1_C_SET_R = OLD_CR_R;
    public static double P2_R1_C_KICK_L = OLD_CR_L, P2_R1_C_KICK_R = OLD_CR_R;
    public static double P2_R1_C_POST_L = OLD_CR_L, P2_R1_C_POST_R = OLD_CR_R;

    public static double P2_R2_A_MOVE_L = 0.0, P2_R2_A_MOVE_R = 0.0;
    public static double P2_R2_A_SET_L  = OLD_CR_L, P2_R2_A_SET_R = OLD_CR_R;
    public static double P2_R2_A_KICK_L = OLD_CR_L, P2_R2_A_KICK_R = OLD_CR_R;
    public static double P2_R2_A_POST_L = OLD_CR_L, P2_R2_A_POST_R = OLD_CR_R;

    public static double P2_R2_B_MOVE_L = 0.0, P2_R2_B_MOVE_R = 0.0;
    public static double P2_R2_B_SET_L  = OLD_CR_L, P2_R2_B_SET_R = OLD_CR_R;
    public static double P2_R2_B_KICK_L = OLD_CR_L, P2_R2_B_KICK_R = OLD_CR_R;
    public static double P2_R2_B_POST_L = OLD_CR_L, P2_R2_B_POST_R = OLD_CR_R;

    public static double P2_R2_C_MOVE_L = 0.0, P2_R2_C_MOVE_R = 0.0;
    public static double P2_R2_C_SET_L  = OLD_CR_L, P2_R2_C_SET_R = OLD_CR_R;
    public static double P2_R2_C_KICK_L = OLD_CR_L, P2_R2_C_KICK_R = OLD_CR_R;
    public static double P2_R2_C_POST_L = OLD_CR_L, P2_R2_C_POST_R = OLD_CR_R;

    /* =========================================================
       =================== BASELINE CONTENT ====================
       (pathing + intake is copied and unchanged)
       ========================================================= */

    /* ================= DRIVE ================= */
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPos = new Pose(32, 135, Math.toRadians(180));
    private final Pose shootPos = new Pose(41, 111.05, Math.toRadians(140));

    private final Pose intakeRowOnePos = new Pose(52, 78, Math.toRadians(0));
    private final Pose intakePickUpRowOnePos = new Pose(22, 78, Math.toRadians(0));

    private final Pose intakeRowTwoPos = new Pose(52, 55, Math.toRadians(0));
    private final Pose intakePickUpRowTwoPos = new Pose(24.5 , 55, Math.toRadians(0));

    private PathChain moveOne, moveTwo, moveThree, moveFour;
    private PathChain moveFive, moveSix, moveSeven;

    /* ================= HARDWARE ================= */
    private DcMotorEx intake, shooter;
    private CRServo crLeft, crRight;
    private Servo spinServo, kickServo, adjustServo;
    private DistanceSensor distanceSensor;

    /* ================= PWM ================= */
    public static int PWM_MIN = 800;
    public static int PWM_MAX = 2200;

    /* ================= INTAKE ================= */
    public static double INTAKE_POWER = 0.8;
    public static double CR_INTAKE_POWER = 1.0;
    public static double DISTANCE_CM = 2.0;

    public static double INTAKE_0 = 0.145;
    public static double INTAKE_1 = 0.41;
    public static double INTAKE_2 = 0.7;

    private int ballCount = 0;
    private boolean lastDetected = false;
    private final ElapsedTime detectTimer = new ElapsedTime();

    /* ================= SHOOTER ================= */
    public static double TICKS_PER_REV = 28.0;
    public static double RPM = 2390;
    public static double RPM_TOL = 75;
    public static double HOOD_POS = 0.75;

    /* ================= KICKER ================= */
    public static double KICK_IDLE = 1.0;
    public static double KICK_ACTIVE = 0.7;
    public static double SETTLE_SEC = 0.35;
    public static double KICK_MS = 500;
    public static double POST_MS = 500;

    /* ================= STATE ================= */
    private enum Mode { IDLE, SHOOT }
    private Mode mode = Mode.IDLE;

    /** Which phase of shooting we are in (for pattern + CR tables) */
    private enum ShootPhase { PRELOAD, ROW1, ROW2 }
    private ShootPhase shootPhase = ShootPhase.PRELOAD;

    /** Shoot FSM states */
    private enum ShootState { START_SHOT, SETTLE, KICK, POST, DONE }
    private ShootState shootState = ShootState.DONE;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private boolean prespin = false;

    /** Current sequence and index */
    private String currentSeq = "ABC";
    private int seqIndex = 0;
    private char currentLetter = 'A';

    /** Track if we just issued a new letter (MOVE stage) */
    private boolean inMoveStage = false;

    /* ================= PATHS ================= */
    public void buildPaths() {
        moveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowOnePos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowOnePos.getHeading())
                .build();

        moveThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowOnePos), new Point(intakePickUpRowOnePos)))
                .setLinearHeadingInterpolation(intakeRowOnePos.getHeading(), intakePickUpRowOnePos.getHeading())
                .build();

        moveFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePickUpRowOnePos), new Point(shootPos)))
                .setLinearHeadingInterpolation(intakePickUpRowOnePos.getHeading(), shootPos.getHeading())
                .build();

        moveFive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPos), new Point(intakeRowTwoPos)))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeRowTwoPos.getHeading())
                .build();

        moveSix = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeRowTwoPos), new Point(intakePickUpRowTwoPos)))
                .setLinearHeadingInterpolation(intakeRowTwoPos.getHeading(), intakePickUpRowTwoPos.getHeading())
                .build();

        moveSeven = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePickUpRowTwoPos), new Point(shootPos)))
                .setLinearHeadingInterpolation(intakePickUpRowTwoPos.getHeading(), shootPos.getHeading())
                .build();
    }

    /* ================= AUTO FSM (UNCHANGED) ================= */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Drive to shoot
                prespin = true;
                setShooterRPM(RPM);
                adjustServo.setPosition(HOOD_POS);
                follower.followPath(moveOne);
                setPathState(1);
                break;

            case 1: // Shoot preload
                if (!follower.isBusy()) {
                    prespin = false;
                    shootPhase = ShootPhase.PRELOAD;
                    startShoot(RPM, HOOD_POS, shootPhase);
                    setPathState(2);
                }
                break;

            case 2: // After preload shot → Row 1
                updateMechanisms();
                if (mode == Mode.IDLE) {
                    ballCount = 0;
                    follower.followPath(moveTwo);
                    setPathState(3);
                }
                break;

            case 3: // Start Row 1 intake
                if (!follower.isBusy()) {
                    prespin = true;
                    setShooterRPM(RPM);
                    follower.setMaxPower(0.2);
                    follower.followPath(moveThree);
                    setPathState(4);
                }
                break;

            case 4: // Intake Row 1 (UNCHANGED)
                prespin = true;
                setShooterRPM(RPM);
                runIntake();

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    stopIntake();
                    follower.followPath(moveFour);
                    setPathState(5);
                }
                break;

            case 5: // Shoot Row 1
                if (!follower.isBusy()) {
                    prespin = false;
                    shootPhase = ShootPhase.ROW1;
                    startShoot(RPM, HOOD_POS, shootPhase);
                    setPathState(6);
                }
                break;

            case 6: // After Row 1 shot → Row 2
                updateMechanisms();
                if (mode == Mode.IDLE) {
                    ballCount = 0;
                    follower.followPath(moveFive);
                    setPathState(7);
                }
                break;

            case 7: // Start Row 2 intake
                if (!follower.isBusy()) {
                    prespin = true;
                    setShooterRPM(RPM);
                    follower.setMaxPower(0.2);
                    follower.followPath(moveSix);
                    setPathState(8);
                }
                break;

            case 8: // Intake Row 2 (UNCHANGED)
                prespin = true;
                setShooterRPM(RPM);
                runIntake();

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    stopIntake();
                    follower.followPath(moveSeven);
                    setPathState(9);
                }
                break;

            case 9: // Shoot Row 2
                if (!follower.isBusy()) {
                    prespin = false;
                    shootPhase = ShootPhase.ROW2;
                    startShoot(RPM, HOOD_POS, shootPhase);
                    setPathState(10);
                }
                break;

            case 10: // End
                updateMechanisms();
                if (mode == Mode.IDLE) setPathState(-1);
                break;
        }
    }

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }

    /* ================= INIT ================= */
    @Override
    public void init() {
        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        buildPaths();

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(70, 0, 8, 13.5)
        );

        crLeft = hardwareMap.get(CRServo.class, "cr_left");
        crRight = hardwareMap.get(CRServo.class, "cr_right");
        crRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spinServo = hardwareMap.get(Servo.class, "spin");
        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        adjustServo = hardwareMap.get(Servo.class, "adjust_servo");

        if (spinServo instanceof PwmControl)
            ((PwmControl) spinServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        if (adjustServo instanceof PwmControl)
            ((PwmControl) adjustServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_left");

        kickServo.setPosition(KICK_IDLE);
        spinServo.setPosition(SHOOT_A);
        adjustServo.setPosition(HOOD_POS);

        // Start safe
        setCR(0, 0);
        setShooterRPM(0);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Pattern", SELECTED_PATTERN);
        telemetry.addData("Phase", shootPhase);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Mode", mode);
        telemetry.addData("Seq", currentSeq);
        telemetry.addData("SeqIndex", seqIndex);
        telemetry.addData("Letter", String.valueOf(currentLetter));
        telemetry.addData("ShootState", shootState);
        telemetry.addData("RPM", getShooterRPM());
        telemetry.addData("Ball Count", ballCount);
        telemetry.update();
    }

    /* ================= MECHANISMS ================= */

    private void startShoot(double rpm, double hood, ShootPhase phase) {
        mode = Mode.SHOOT;
        shootPhase = phase;

        adjustServo.setPosition(hood);
        setShooterRPM(rpm);

        currentSeq = getSequenceFor(SELECTED_PATTERN, phase);
        seqIndex = 0;

        shootState = ShootState.START_SHOT;
        shootTimer.reset();
    }

    private void updateMechanisms() {
        if (mode == Mode.SHOOT) runShoot();
        else hold();
    }

    /* ================= INTAKE (UNCHANGED) ================= */

    private void runIntake() {
        if (ballCount >= 3) {
            stopIntake();
            return;
        }

        intake.setPower(INTAKE_POWER);
        setCR(CR_INTAKE_POWER, CR_INTAKE_POWER);

        boolean detected = distanceSensor.getDistance(DistanceUnit.CM) <= DISTANCE_CM;
        if (detected && !lastDetected && detectTimer.seconds() > 0.4) {
            ballCount++;
            detectTimer.reset();
        }
        lastDetected = detected;

        spinServo.setPosition(
                ballCount == 0 ? INTAKE_0 :
                        ballCount == 1 ? INTAKE_1 : INTAKE_2
        );
    }

    private void stopIntake() {
        intake.setPower(0);
        setCR(0, 0);
    }

    /* ================= SHOOT FSM (PATTERN + STAGE CR) ================= */

    private void runShoot() {
        if (seqIndex >= currentSeq.length()) {
            // done with this phase
            setCR(0, 0);
            setShooterRPM(0);
            spinServo.setPosition(INTAKE_0);
            kickServo.setPosition(KICK_IDLE);
            mode = Mode.IDLE;
            shootState = ShootState.DONE;
            return;
        }

        switch (shootState) {

            case START_SHOT: {
                currentLetter = currentSeq.charAt(seqIndex);

                // MOVE stage: command spin + CR for MOVE
                spinServo.setPosition(getSpinFor(currentLetter));
                applyCR(shootPhase, currentLetter, Stage.MOVE);

                // We treat MOVE as instantaneous (servo moves while we "settle")
                inMoveStage = true;
                shootTimer.reset();
                shootState = ShootState.SETTLE;
                break;
            }

            case SETTLE: {
                // Once we enter SETTLE, apply SETTLE stage CR (only once)
                if (inMoveStage) {
                    applyCR(shootPhase, currentLetter, Stage.SETTLE);
                    inMoveStage = false;
                }

                if (shootTimer.seconds() >= SETTLE_SEC && shooterAtSpeed()) {
                    applyCR(shootPhase, currentLetter, Stage.KICK);
                    kickServo.setPosition(KICK_ACTIVE);
                    shootTimer.reset();
                    shootState = ShootState.KICK;
                }
                break;
            }

            case KICK: {
                if (shootTimer.milliseconds() >= KICK_MS) {
                    kickServo.setPosition(KICK_IDLE);
                    applyCR(shootPhase, currentLetter, Stage.POST);
                    shootTimer.reset();
                    shootState = ShootState.POST;
                }
                break;
            }

            case POST: {
                if (shootTimer.milliseconds() >= POST_MS) {
                    seqIndex++;
                    shootState = ShootState.START_SHOT;
                }
                break;
            }

            case DONE:
            default:
                break;
        }
    }

    private void hold() {
        stopIntake(); // same as baseline behavior
        if (!prespin) setShooterRPM(0);
        kickServo.setPosition(KICK_IDLE);
    }

    /* ================= HELPERS ================= */

    private void setCR(double l, double r) {
        crLeft.setPower(l);
        crRight.setPower(r);
    }

    private void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * TICKS_PER_REV) / 60.0);
    }

    private double getShooterRPM() {
        return shooter.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    private boolean shooterAtSpeed() {
        return Math.abs(getShooterRPM() - RPM) < RPM_TOL;
    }

    private double getSpinFor(char letter) {
        switch (letter) {
            case 'A': return SHOOT_A;
            case 'B': return SHOOT_B;
            case 'C': return SHOOT_C;
            default:  return SHOOT_A;
        }
    }

    private String getSequenceFor(int pattern, ShootPhase phase) {
        if (pattern == 1) {
            if (phase == ShootPhase.PRELOAD) return P1_PRELOAD;
            if (phase == ShootPhase.ROW1)    return P1_ROW1;
            return P1_ROW2;
        } else if (pattern == 2) {
            if (phase == ShootPhase.PRELOAD) return P2_PRELOAD;
            if (phase == ShootPhase.ROW1)    return P2_ROW1;
            return P2_ROW2;
        } else {
            if (phase == ShootPhase.PRELOAD) return P0_PRELOAD;
            if (phase == ShootPhase.ROW1)    return P0_ROW1;
            return P0_ROW2;
        }
    }

    private void applyCR(ShootPhase phase, char letter, Stage stage) {
        double l = 0, r = 0;

        // Pattern switch
        int p = SELECTED_PATTERN;

        if (p == 0) {
            l = lookupCR_P0(phase, letter, stage, true);
            r = lookupCR_P0(phase, letter, stage, false);
        } else if (p == 1) {
            l = lookupCR_P1(phase, letter, stage, true);
            r = lookupCR_P1(phase, letter, stage, false);
        } else {
            l = lookupCR_P2(phase, letter, stage, true);
            r = lookupCR_P2(phase, letter, stage, false);
        }

        setCR(l, r);
    }

    private double lookupCR_P0(ShootPhase phase, char letter, Stage stage, boolean left) {
        // PRELOAD
        if (phase == ShootPhase.PRELOAD) {
            if (letter == 'A') return pick(left, stage, P0_PRE_A_MOVE_L, P0_PRE_A_MOVE_R, P0_PRE_A_SET_L, P0_PRE_A_SET_R, P0_PRE_A_KICK_L, P0_PRE_A_KICK_R, P0_PRE_A_POST_L, P0_PRE_A_POST_R);
            if (letter == 'B') return pick(left, stage, P0_PRE_B_MOVE_L, P0_PRE_B_MOVE_R, P0_PRE_B_SET_L, P0_PRE_B_SET_R, P0_PRE_B_KICK_L, P0_PRE_B_KICK_R, P0_PRE_B_POST_L, P0_PRE_B_POST_R);
            return pick(left, stage, P0_PRE_C_MOVE_L, P0_PRE_C_MOVE_R, P0_PRE_C_SET_L, P0_PRE_C_SET_R, P0_PRE_C_KICK_L, P0_PRE_C_KICK_R, P0_PRE_C_POST_L, P0_PRE_C_POST_R);
        }
        // ROW1
        if (phase == ShootPhase.ROW1) {
            if (letter == 'A') return pick(left, stage, P0_R1_A_MOVE_L, P0_R1_A_MOVE_R, P0_R1_A_SET_L, P0_R1_A_SET_R, P0_R1_A_KICK_L, P0_R1_A_KICK_R, P0_R1_A_POST_L, P0_R1_A_POST_R);
            if (letter == 'B') return pick(left, stage, P0_R1_B_MOVE_L, P0_R1_B_MOVE_R, P0_R1_B_SET_L, P0_R1_B_SET_R, P0_R1_B_KICK_L, P0_R1_B_KICK_R, P0_R1_B_POST_L, P0_R1_B_POST_R);
            return pick(left, stage, P0_R1_C_MOVE_L, P0_R1_C_MOVE_R, P0_R1_C_SET_L, P0_R1_C_SET_R, P0_R1_C_KICK_L, P0_R1_C_KICK_R, P0_R1_C_POST_L, P0_R1_C_POST_R);
        }
        // ROW2
        if (letter == 'A') return pick(left, stage, P0_R2_A_MOVE_L, P0_R2_A_MOVE_R, P0_R2_A_SET_L, P0_R2_A_SET_R, P0_R2_A_KICK_L, P0_R2_A_KICK_R, P0_R2_A_POST_L, P0_R2_A_POST_R);
        if (letter == 'B') return pick(left, stage, P0_R2_B_MOVE_L, P0_R2_B_MOVE_R, P0_R2_B_SET_L, P0_R2_B_SET_R, P0_R2_B_KICK_L, P0_R2_B_KICK_R, P0_R2_B_POST_L, P0_R2_B_POST_R);
        return pick(left, stage, P0_R2_C_MOVE_L, P0_R2_C_MOVE_R, P0_R2_C_SET_L, P0_R2_C_SET_R, P0_R2_C_KICK_L, P0_R2_C_KICK_R, P0_R2_C_POST_L, P0_R2_C_POST_R);
    }

    private double lookupCR_P1(ShootPhase phase, char letter, Stage stage, boolean left) {
        if (phase == ShootPhase.PRELOAD) {
            if (letter == 'A') return pick(left, stage, P1_PRE_A_MOVE_L, P1_PRE_A_MOVE_R, P1_PRE_A_SET_L, P1_PRE_A_SET_R, P1_PRE_A_KICK_L, P1_PRE_A_KICK_R, P1_PRE_A_POST_L, P1_PRE_A_POST_R);
            if (letter == 'B') return pick(left, stage, P1_PRE_B_MOVE_L, P1_PRE_B_MOVE_R, P1_PRE_B_SET_L, P1_PRE_B_SET_R, P1_PRE_B_KICK_L, P1_PRE_B_KICK_R, P1_PRE_B_POST_L, P1_PRE_B_POST_R);
            return pick(left, stage, P1_PRE_C_MOVE_L, P1_PRE_C_MOVE_R, P1_PRE_C_SET_L, P1_PRE_C_SET_R, P1_PRE_C_KICK_L, P1_PRE_C_KICK_R, P1_PRE_C_POST_L, P1_PRE_C_POST_R);
        }
        if (phase == ShootPhase.ROW1) {
            if (letter == 'A') return pick(left, stage, P1_R1_A_MOVE_L, P1_R1_A_MOVE_R, P1_R1_A_SET_L, P1_R1_A_SET_R, P1_R1_A_KICK_L, P1_R1_A_KICK_R, P1_R1_A_POST_L, P1_R1_A_POST_R);
            if (letter == 'B') return pick(left, stage, P1_R1_B_MOVE_L, P1_R1_B_MOVE_R, P1_R1_B_SET_L, P1_R1_B_SET_R, P1_R1_B_KICK_L, P1_R1_B_KICK_R, P1_R1_B_POST_L, P1_R1_B_POST_R);
            return pick(left, stage, P1_R1_C_MOVE_L, P1_R1_C_MOVE_R, P1_R1_C_SET_L, P1_R1_C_SET_R, P1_R1_C_KICK_L, P1_R1_C_KICK_R, P1_R1_C_POST_L, P1_R1_C_POST_R);
        }
        if (letter == 'A') return pick(left, stage, P1_R2_A_MOVE_L, P1_R2_A_MOVE_R, P1_R2_A_SET_L, P1_R2_A_SET_R, P1_R2_A_KICK_L, P1_R2_A_KICK_R, P1_R2_A_POST_L, P1_R2_A_POST_R);
        if (letter == 'B') return pick(left, stage, P1_R2_B_MOVE_L, P1_R2_B_MOVE_R, P1_R2_B_SET_L, P1_R2_B_SET_R, P1_R2_B_KICK_L, P1_R2_B_KICK_R, P1_R2_B_POST_L, P1_R2_B_POST_R);
        return pick(left, stage, P1_R2_C_MOVE_L, P1_R2_C_MOVE_R, P1_R2_C_SET_L, P1_R2_C_SET_R, P1_R2_C_KICK_L, P1_R2_C_KICK_R, P1_R2_C_POST_L, P1_R2_C_POST_R);
    }

    private double lookupCR_P2(ShootPhase phase, char letter, Stage stage, boolean left) {
        if (phase == ShootPhase.PRELOAD) {
            if (letter == 'A') return pick(left, stage, P2_PRE_A_MOVE_L, P2_PRE_A_MOVE_R, P2_PRE_A_SET_L, P2_PRE_A_SET_R, P2_PRE_A_KICK_L, P2_PRE_A_KICK_R, P2_PRE_A_POST_L, P2_PRE_A_POST_R);
            if (letter == 'B') return pick(left, stage, P2_PRE_B_MOVE_L, P2_PRE_B_MOVE_R, P2_PRE_B_SET_L, P2_PRE_B_SET_R, P2_PRE_B_KICK_L, P2_PRE_B_KICK_R, P2_PRE_B_POST_L, P2_PRE_B_POST_R);
            return pick(left, stage, P2_PRE_C_MOVE_L, P2_PRE_C_MOVE_R, P2_PRE_C_SET_L, P2_PRE_C_SET_R, P2_PRE_C_KICK_L, P2_PRE_C_KICK_R, P2_PRE_C_POST_L, P2_PRE_C_POST_R);
        }
        if (phase == ShootPhase.ROW1) {
            if (letter == 'A') return pick(left, stage, P2_R1_A_MOVE_L, P2_R1_A_MOVE_R, P2_R1_A_SET_L, P2_R1_A_SET_R, P2_R1_A_KICK_L, P2_R1_A_KICK_R, P2_R1_A_POST_L, P2_R1_A_POST_R);
            if (letter == 'B') return pick(left, stage, P2_R1_B_MOVE_L, P2_R1_B_MOVE_R, P2_R1_B_SET_L, P2_R1_B_SET_R, P2_R1_B_KICK_L, P2_R1_B_KICK_R, P2_R1_B_POST_L, P2_R1_B_POST_R);
            return pick(left, stage, P2_R1_C_MOVE_L, P2_R1_C_MOVE_R, P2_R1_C_SET_L, P2_R1_C_SET_R, P2_R1_C_KICK_L, P2_R1_C_KICK_R, P2_R1_C_POST_L, P2_R1_C_POST_R);
        }
        if (letter == 'A') return pick(left, stage, P2_R2_A_MOVE_L, P2_R2_A_MOVE_R, P2_R2_A_SET_L, P2_R2_A_SET_R, P2_R2_A_KICK_L, P2_R2_A_KICK_R, P2_R2_A_POST_L, P2_R2_A_POST_R);
        if (letter == 'B') return pick(left, stage, P2_R2_B_MOVE_L, P2_R2_B_MOVE_R, P2_R2_B_SET_L, P2_R2_B_SET_R, P2_R2_B_KICK_L, P2_R2_B_KICK_R, P2_R2_B_POST_L, P2_R2_B_POST_R);
        return pick(left, stage, P2_R2_C_MOVE_L, P2_R2_C_MOVE_R, P2_R2_C_SET_L, P2_R2_C_SET_R, P2_R2_C_KICK_L, P2_R2_C_KICK_R, P2_R2_C_POST_L, P2_R2_C_POST_R);
    }

    private double pick(boolean left, Stage stage,
                        double moveL, double moveR,
                        double setL,  double setR,
                        double kickL, double kickR,
                        double postL, double postR) {
        switch (stage) {
            case MOVE:   return left ? moveL : moveR;
            case SETTLE: return left ? setL  : setR;
            case KICK:   return left ? kickL : kickR;
            case POST:   return left ? postL : postR;
            default:     return 0.0;
        }
    }
}
