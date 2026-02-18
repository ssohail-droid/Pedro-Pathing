//package pedroPathing.examples;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//import pedroPathing.SubSystem.ShooterSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//import java.io.BufferedWriter;
//import java.io.File;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.List;
//
///**
// * ═══════════════════════════════════════════════════════════════════════════
// * LIMELIGHT 3A INTEGRATED - BlueAUTO ML Enhanced v10
// * DECODE 2025-2026 FTC Season
// * ═══════════════════════════════════════════════════════════════════════════
// *
// * 🎯 LIMELIGHT 3A FEATURES:
// * ✓ AprilTag Pose Correction (eliminates odometry drift)
// * ✓ Artifact Detection (GREEN and PURPLE game pieces)
// * ✓ Distance-Based Adaptive Shooting (RPM adjusts to distance)
// * ✓ Real-time Vision Telemetry
// * ✓ Neural Network Object Detection (if configured)
// *
// * 🧠 MACHINE LEARNING FEATURES:
// * ✓ Adaptive RPM based on battery voltage
// * ✓ Predictive shot timing
// * ✓ Smart timeout adjustment
// * ✓ Pose error correction (Limelight + Odometry fusion)
// * ✓ Battery-aware speed control
// * ✓ Data logging for offline training
// *
// * 📋 HARDWARE REQUIREMENTS:
// * - Limelight 3A (mounted, angled 15-30° down)
// * - GoBilda Pinpoint Odometry
// * - REV Color Sensor V3 (optional, for transfer belt)
// * - Shooter motor with encoder
// * - Intake + Transfer motors
// * - Push servo + Hold servo
// *
// * 🔌 LIMELIGHT WIRING:
// * - Power: 12V XT30 from REV Hub
// * - Data: Ethernet to Control Hub
// * - IP: 192.168.43.11 (configured in Limelight web interface)
// *
// * 📊 LIMELIGHT PIPELINES:
// * Pipeline 0: AprilTag Detection (pose correction)
// * Pipeline 1: Artifact Detection - GREEN (alliance artifacts)
// * Pipeline 2: Artifact Detection - PURPLE (alliance artifacts)
// *
// * 🎮 DECODE GAME ELEMENTS:
// * - Classifier: High scoring zone (requires shooting)
// * - Depot: Low scoring zone
// * - Artifacts: GREEN and PURPLE game pieces to score
// * - Gates: Autonomous navigation zones
// * - Secret Tunnel: Bonus navigation path
// *
// * ═══════════════════════════════════════════════════════════════════════════
// */
//
//@Config
//@Autonomous(name = "BlueAUTO DECODE v10", group = "Competition")
//public class BlueAUTO_ML_Limelight extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//    private int pathState;
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // LIMELIGHT CONFIGURATION (FTC Dashboard Adjustable)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    public static boolean ENABLE_LIMELIGHT = true;
//    public static boolean ENABLE_APRILTAG_CORRECTION = true;
//    public static boolean ENABLE_ARTIFACT_DETECTION = false; // Enable for dynamic pickup
//    public static boolean ENABLE_DISTANCE_SHOOTING = true; // Adjust RPM by distance
//
//    // AprilTag Pose Correction Settings
//    public static double APRILTAG_CORRECTION_INTERVAL = 2.0; // How often to correct pose (seconds)
//    public static double APRILTAG_TRUST_FACTOR = 0.7; // 0.0-1.0 (higher = trust Limelight more than odometry)
//    public static double MAX_APRILTAG_DISTANCE = 120.0; // Ignore AprilTags farther than this (inches)
//
//    // Artifact Detection Settings (GREEN and PURPLE)
//    public static int GREEN_ARTIFACT_PIPELINE = 1; // Limelight pipeline for green artifacts
//    public static int PURPLE_ARTIFACT_PIPELINE = 2; // Limelight pipeline for purple artifacts
//    public static double ARTIFACT_MIN_AREA = 0.5; // Minimum artifact size (% of image)
//    public static double ARTIFACT_MAX_TX = 15.0; // Ignore artifacts outside this horizontal angle (degrees)
//
//    // Distance-Based Shooting Settings (tune these for your robot)
//    public static double RPM_PER_INCH = 5.0; // How much RPM increases per inch of distance
//    public static double BASE_SHOOTING_DISTANCE = 24.0; // Baseline shooting distance (inches)
//    public static double MIN_SHOOTING_RPM = 2200.0; // Minimum safe RPM
//    public static double MAX_SHOOTING_RPM = 2800.0; // Maximum safe RPM
//
//    // Limelight Physical Mounting (MEASURE AND ADJUST FOR YOUR ROBOT)
//    public static double LIMELIGHT_HEIGHT_INCHES = 12.0; // Height of Limelight off ground
//    public static double LIMELIGHT_ANGLE_DEGREES = 20.0; // UPWARD tilt angle (positive = up)
//    public static double LIMELIGHT_FORWARD_OFFSET = 6.0; // Distance from robot center (inches)
//
//    // DECODE Scoring Zone Heights (ADJUST IF NEEDED)
//    public static double CLASSIFIER_HEIGHT_INCHES = 48.0; // Height of Classifier scoring zone
//    public static double DEPOT_HEIGHT_INCHES = 12.0; // Height of Depot scoring zone
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // ML CONFIGURATION (FTC Dashboard Adjustable)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    public static boolean ENABLE_ADAPTIVE_RPM = true;
//    public static double RPM_VOLTAGE_COEFFICIENT = 15.0; // RPM adjustment per volt
//    public static double BASE_RPM = 2500.0; // Base RPM at 12.5V
//
//    public static boolean ENABLE_PREDICTIVE_TIMING = true;
//    public static double LEARNED_TRANSFER_TIME_MS = 250.0; // Learned delay before transfer starts
//
//    public static boolean ENABLE_SMART_TIMEOUTS = true;
//    public static double RPM_MONITOR_TIMEOUT_MS = 5000.0; // Max time to wait for RPM dip
//
//    public static boolean ENABLE_POSE_CORRECTION = true;
//    public static double X_DRIFT_CORRECTION = 0.0; // Manual X position correction (inches)
//    public static double Y_DRIFT_CORRECTION = 0.0; // Manual Y position correction (inches)
//
//    public static boolean ENABLE_BATTERY_COMPENSATION = true;
//    public static double MIN_VOLTAGE_THRESHOLD = 11.5; // Reduce speed below this voltage
//
//    public static boolean ENABLE_DATA_LOGGING = true; // Log data for ML training
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // SHOOTING CYCLE CONFIGURATION (NEW - Dashboard Adjustable)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    public static double HOLD_RETRACT_DELAY_MS = 100.0; // Delay after retracting hold servo
//    public static double RPM_SPINUP_THRESHOLD = 0.95; // % of target RPM before shooting (95%)
//    public static double RPM_DIP_THRESHOLD = 0.92; // % of target RPM to detect shot (92%)
//    public static double INTAKE_START_DELAY_MS = 200.0; // Delay before starting intake after hold retract
//    public static double HOLD_CLOSE_PREDICTIVE_MS = 150.0; // Predictive timing to close hold servo
//    public static double PUSH_SERVO_DURATION_MS = 2000.0; // How long to engage push servo on 3rd shot
//    public static double SHOT_RECOVERY_DELAY_MS = 500.0; // Delay between shots for RPM recovery
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // AUTONOMOUS PATH CONFIGURATION (EDITED TO MATCH YOUR 5-WAYPOINT PLAN)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    @Config
//    public static class Waypoints {
//        // Start (at goal, shooter looking directly at the goal)
//        // From screenshot panel: Current Robot Position ~ X=21, Y=122.5, Heading≈144°
//        public static double START_X = 21.0;
//        public static double START_Y = 122.5;
//        public static double START_HDG_DEG = 144.0;
//
//        // Path 1 end: X=60.5, Y=100, heading 144
//        public static double P1_X = 60.5;
//        public static double P1_Y = 100.0;
//        public static double P1_HDG_DEG = 144.0;
//
//        // Path 2 end: X=60.5, Y=84, heading 143 (face the 3-ball row with intake)
//        public static double P2_X = 60.5;
//        public static double P2_Y = 84.0;
//        public static double P2_HDG_DEG = 143.0;
//
//        // Path 3 end: X=15, Y=84, heading 0 (slow sweep across the 3 balls)
//        public static double P3_X = 15.0;
//        public static double P3_Y = 84.0;
//        public static double P3_HDG_DEG = 0.0;
//
//        // Path 4 end: X=60.5, Y=84, heading 0 (back across, still intaking)
//        public static double P4_X = 60.5;
//        public static double P4_Y = 84.0;
//        public static double P4_HDG_DEG = 0.0;
//
//        // Path 5 end: X=60.5, Y=100, heading 143 (back to shoot)
//        public static double P5_X = 60.5;
//        public static double P5_Y = 100.0;
//        public static double P5_HDG_DEG = 143.0;
//
//        // Segment speeds (Dashboard tunable)
//        public static double P1_MAX_POWER = 0.40;
//        public static double P2_MAX_POWER = 0.35;
//        public static double P3_MAX_POWER = 0.22; // slow sweep
//        public static double P4_MAX_POWER = 0.30;
//        public static double P5_MAX_POWER = 0.35;
//
//        // Shot counts
//        public static int PRELOAD_SHOTS = 3; // after Path1
//        public static int CYCLE_SHOTS = 3;   // after Path5
//    }
//
//    // If your planner/simulator heading needs (+180)%360 to match robot-forward,
//    // set this to true (Dashboard-tunable).
//    public static boolean USE_HEADING_PLUS_180 = false;
//
//    private double toHeadingRad(double deg) {
//        double h = USE_HEADING_PLUS_180 ? ((deg + 180.0) % 360.0) : deg;
//        return Math.toRadians(h);
//    }
//
//    // Five-Point Sequence
//    private Pose startPose, p1Pose, p2Pose, p3Pose, p4Pose, p5Pose;
//    private PathChain path1, path2, path3, path4, path5;
//
//    // Legacy placeholders retained for compatibility with ML logging
//    private final int Heading = 180; // Robot heading (degrees)
//    private final Pose onePos = new Pose(134.3, 34, Math.toRadians(Heading)); // Starting position (not used for paths)
//    private final Pose twoPos = new Pose(118.5, 34, Math.toRadians(Heading)); // Shooting position (not used for paths)
//
//    // Subsystems
//    private IntakeSubsystem intake;
//    private ShooterSubsystem shooter;
//    private TransferSubsystem transfer;
//    private ServoSubsystem servos;
//
//    // Shooting state machine variables (UPDATED FOR NEW 3-BALL SEQUENCE)
//    private int shotStep = 0; // Current step in shooting sequence
//    private int totalShots = 4; // kept for backward compatibility
//    private int currentShot = 0; // Current shot number (1, 2, or 3)
//    private boolean rpmDipped = false; // Did RPM dip (indicating shot fired)?
//    private long stepStartTime = 0; // Time when current step started
//    private double targetRPMForShot = 0; // Target RPM for current shot
//
//    // New: multi-shot helper controls
//    private int shotsTarget = 0;
//    private boolean shotsRunning = false;
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // LIMELIGHT RUNTIME VARIABLES
//    // ═══════════════════════════════════════════════════════════════════════
//
//    private Limelight3A limelight;
//    private double lastAprilTagCorrectionTime = 0;
//    private boolean limelightConnected = false;
//    private int aprilTagCorrections = 0; // Count of pose corrections
//    private int greenArtifactsDetected = 0; // Count of green artifacts detected
//    private int purpleArtifactsDetected = 0; // Count of purple artifacts detected
//
//    // Current vision data
//    private double currentDistanceToTarget = 0; // Distance to target (inches)
//    private boolean hasValidTarget = false; // Is a target currently visible?
//    private double targetTx = 0; // Horizontal offset to target (degrees)
//    private double targetTy = 0; // Vertical offset to target (degrees)
//    private double targetArea = 0; // Target size (% of image)
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // ML RUNTIME VARIABLES
//    // ═══════════════════════════════════════════════════════════════════════
//
//    private VoltageSensor voltageSensor;
//    private double currentVoltage = 12.5;
//    private double adaptiveMaxPower = 0.4; // Dynamically adjusted max power
//    private MultipleTelemetry multiTelemetry;
//
//    // ML data logging
//    private List<MLDataPoint> trainingData = new ArrayList<>();
//    private long shotStartTime = 0;
//    private Pose shotStartPose;
//
//    // Performance tracking
//    private int successfulShots = 0; // Shots that detected RPM dip
//    private int timeoutShots = 0; // Shots that timed out
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // ML DATA STRUCTURE (for training data export)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    private static class MLDataPoint {
//        double voltage;
//        double targetRPM;
//        double actualRPM;
//        double shotDuration;
//        boolean rpmDipped;
//        boolean timeout;
//        double xError;
//        double yError;
//        double distanceToTarget;
//        boolean usedLimelight;
//        long timestamp;
//
//        @Override
//        public String toString() {
//            return String.format("%.2f,%.1f,%.1f,%.0f,%b,%b,%.2f,%.2f,%.1f,%b,%d",
//                    voltage, targetRPM, actualRPM, shotDuration, rpmDipped, timeout,
//                    xError, yError, distanceToTarget, usedLimelight, timestamp);
//        }
//    }
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // LIMELIGHT FUNCTIONS
//    // ═══════════════════════════════════════════════════════════════════════
//
//    /**
//     * Initialize Limelight 3A hardware
//     * Attempts to connect to Limelight and start AprilTag pipeline
//     */
//    private void initializeLimelight() {
//        try {
//            limelight = hardwareMap.get(Limelight3A.class, "limelight");
//            limelight.pipelineSwitch(0); // Pipeline 0 = AprilTag detection
//            limelight.start();
//            limelightConnected = true;
//
//            multiTelemetry.addLine("✓ Limelight 3A Connected!");
//            multiTelemetry.addData("Status", limelight.getStatus());
//        } catch (Exception e) {
//            limelightConnected = false;
//            multiTelemetry.addLine("⚠ Limelight NOT found - running without vision");
//            multiTelemetry.addData("Error", e.getMessage());
//        }
//    }
//
//    /**
//     * Update vision data from Limelight (call every loop)
//     * Reads latest AprilTag or artifact detection results
//     */
//    private void updateLimelightData() {
//        if (!ENABLE_LIMELIGHT || !limelightConnected) return;
//
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            hasValidTarget = true;
//
//            // Check for AprilTag detections
//            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
//                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
//                targetTx = fiducial.getTargetXDegrees();
//                targetTy = fiducial.getTargetYDegrees();
//                targetArea = fiducial.getTargetArea();
//
//                // Calculate distance using trigonometry
//                currentDistanceToTarget = calculateDistanceToTarget(targetTy, CLASSIFIER_HEIGHT_INCHES);
//
//                // Check for color/artifact detections (GREEN or PURPLE)
//            } else if (result.getColorResults() != null && !result.getColorResults().isEmpty()) {
//                LLResultTypes.ColorResult color = result.getColorResults().get(0);
//                targetTx = color.getTargetXDegrees();
//                targetTy = color.getTargetYDegrees();
//                targetArea = color.getTargetArea();
//
//                // Track which color was detected based on current pipeline
//                // Note: Artifact counting happens in findNearestArtifact() method instead
//            }
//        } else {
//            hasValidTarget = false;
//        }
//    }
//
//    /**
//     * Calculate distance to target using camera geometry
//     * Formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + targetAngle)
//     *
//     * @param ty Vertical angle to target (degrees)
//     * @param targetHeight Height of target (inches)
//     * @return Distance to target (inches)
//     */
//    private double calculateDistanceToTarget(double ty, double targetHeight) {
//        double angleToTarget = LIMELIGHT_ANGLE_DEGREES + ty;
//        double distance = (targetHeight - LIMELIGHT_HEIGHT_INCHES) / Math.tan(Math.toRadians(angleToTarget));
//        return Math.abs(distance);
//    }
//
//    /**
//     * LIMELIGHT FEATURE #1: AprilTag Pose Correction
//     * Fuses Limelight AprilTag pose with odometry to eliminate drift
//     * Uses weighted average based on APRILTAG_TRUST_FACTOR
//     */
//    private void updatePoseFromAprilTag() {
//        if (!ENABLE_APRILTAG_CORRECTION || !limelightConnected) return;
//
//        double currentTime = opmodeTimer.getElapsedTimeSeconds();
//        if (currentTime - lastAprilTagCorrectionTime < APRILTAG_CORRECTION_INTERVAL) {
//            return; // Don't correct too frequently (reduces jitter)
//        }
//
//        LLResult result = limelight.getLatestResult();
//        if (result == null || !result.isValid()) return;
//
//        // Check for AprilTag detections
//        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//        if (fiducials == null || fiducials.isEmpty()) return;
//
//        // Get robot pose from Limelight (MegaTag2 = multi-tag fusion, more accurate)
//        Pose3D pose3D = result.getBotpose_MT2();
//
//        // Fallback to single tag if MegaTag2 unavailable
//        if (pose3D == null) {
//            pose3D = result.getBotpose();
//        }
//
//        if (pose3D != null) {
//            // Extract pose components (Limelight returns METERS, convert to INCHES)
//            double limelightX = pose3D.getPosition().x; // meters
//            double limelightY = pose3D.getPosition().y; // meters
//            double limelightYaw = pose3D.getOrientation().getYaw(); // degrees
//
//            // Convert Limelight coordinates to FTC field coordinates
//            // Limelight: X=forward, Y=left | FTC: X=right, Y=forward
//            double fieldY = -limelightY * 39.37; // meters to inches
//            double fieldX = limelightX * 39.37;
//            double fieldHeading = Math.toRadians(limelightYaw);
//
//            // Get current odometry pose
//            Pose odomPose = follower.getPose();
//
//            // Fuse poses using weighted average (trust factor)
//            // Higher APRILTAG_TRUST_FACTOR = trust Limelight more
//            double fusedX = (fieldX * APRILTAG_TRUST_FACTOR) + (odomPose.getX() * (1 - APRILTAG_TRUST_FACTOR));
//            double fusedY = (fieldY * APRILTAG_TRUST_FACTOR) + (odomPose.getY() * (1 - APRILTAG_TRUST_FACTOR));
//            double fusedHeading = (fieldHeading * APRILTAG_TRUST_FACTOR) + (odomPose.getHeading() * (1 - APRILTAG_TRUST_FACTOR));
//
//            // Only apply correction if AprilTag is close enough (avoid bad detections)
//            double distanceToTag = Math.sqrt(Math.pow(limelightX, 2) + Math.pow(limelightY, 2)) * 39.37;
//            if (distanceToTag < MAX_APRILTAG_DISTANCE) {
//                follower.setPose(new Pose(fusedX, fusedY, fusedHeading));
//                aprilTagCorrections++;
//                lastAprilTagCorrectionTime = currentTime;
//            }
//        }
//    }
//
//    /**
//     * LIMELIGHT FEATURE #2: Distance-Based Adaptive Shooting
//     * Adjusts shooter RPM based on measured distance to Classifier
//     * Closer = lower RPM, Farther = higher RPM
//     *
//     * @return Calculated target RPM
//     */
//    private double calculateDistanceBasedRPM() {
//        if (!ENABLE_DISTANCE_SHOOTING || !hasValidTarget) {
//            return calculateAdaptiveRPM(); // Fall back to voltage-based RPM
//        }
//
//        // Calculate RPM adjustment based on distance
//        double distanceDelta = currentDistanceToTarget - BASE_SHOOTING_DISTANCE;
//        double distanceRPM = BASE_RPM + (distanceDelta * RPM_PER_INCH);
//
//        // Clamp to safe range
//        distanceRPM = Math.max(MIN_SHOOTING_RPM, Math.min(MAX_SHOOTING_RPM, distanceRPM));
//
//        // Use distance-based RPM if we have a valid target
//        return hasValidTarget ? distanceRPM : calculateAdaptiveRPM();
//    }
//
//    /**
//     * LIMELIGHT FEATURE #3: Artifact Detection (GREEN and PURPLE)
//     * Finds nearest artifact on field for dynamic pickup
//     * Scores artifacts by size and proximity to center
//     *
//     * @param artifactColor "green" or "purple" - which artifact to search for
//     * @return Pose of nearest artifact, or null if none found
//     */
//    private Pose findNearestArtifact(String artifactColor) {
//        if (!ENABLE_ARTIFACT_DETECTION || !limelightConnected) {
//            return null;
//        }
//
//        // Switch to appropriate artifact detection pipeline
//        int targetPipeline = artifactColor.equalsIgnoreCase("green") ?
//                GREEN_ARTIFACT_PIPELINE : PURPLE_ARTIFACT_PIPELINE;
//        limelight.pipelineSwitch(targetPipeline);
//
//        LLResult result = limelight.getLatestResult();
//        if (result == null || !result.isValid()) return null;
//
//        List<LLResultTypes.ColorResult> artifacts = result.getColorResults();
//        if (artifacts == null || artifacts.isEmpty()) return null;
//
//        // Find best artifact (largest area, closest to center)
//        LLResultTypes.ColorResult bestArtifact = null;
//        double bestScore = -1;
//
//        for (LLResultTypes.ColorResult artifact : artifacts) {
//            double tx = artifact.getTargetXDegrees();
//            double area = artifact.getTargetArea();
//
//            // Ignore artifacts too far to the side
//            if (Math.abs(tx) > ARTIFACT_MAX_TX) continue;
//
//            // Score = area * (1 - normalized horizontal offset)
//            double score = area * (1.0 - Math.abs(tx) / 30.0);
//
//            if (score > bestScore && area > ARTIFACT_MIN_AREA) {
//                bestScore = score;
//                bestArtifact = artifact;
//            }
//        }
//
//        if (bestArtifact == null) return null;
//
//        // Calculate artifact position relative to robot
//        double tx = bestArtifact.getTargetXDegrees();
//        double ty = bestArtifact.getTargetYDegrees();
//        double distance = calculateDistanceToTarget(ty, 2.0); // Artifacts are ~2" tall
//
//        // Convert to field coordinates
//        Pose robotPose = follower.getPose();
//        double artifactX = robotPose.getX() + distance * Math.cos(robotPose.getHeading());
//        double artifactY = robotPose.getY() + distance * Math.sin(robotPose.getHeading());
//
//        return new Pose(artifactX, artifactY, robotPose.getHeading());
//    }
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // ML FUNCTIONS
//    // ═══════════════════════════════════════════════════════════════════════
//
//    /**
//     * Calculate adaptive RPM based on battery voltage
//     * Compensates for voltage drop during match
//     *
//     * @return Voltage-adjusted target RPM
//     */
//    private double calculateAdaptiveRPM() {
//        if (!ENABLE_ADAPTIVE_RPM) {
//            return ShooterSubsystem.targetRPM;
//        }
//        double voltageOffset = currentVoltage - 12.5; // Offset from nominal voltage
//        return BASE_RPM + (voltageOffset * RPM_VOLTAGE_COEFFICIENT);
//    }
//
//    /**
//     * Get learned transfer delay (time between shot and transfer start)
//     *
//     * @return Delay in milliseconds
//     */
//    private long getPredictiveTransferDelay() {
//        if (!ENABLE_PREDICTIVE_TIMING) {
//            return 250; // Default delay
//        }
//        return (long) LEARNED_TRANSFER_TIME_MS;
//    }
//
//    /**
//     * Calculate smart timeout based on historical performance
//     * Increases timeout if many shots are timing out
//     *
//     * @return Timeout in milliseconds
//     */
//    private long getSmartTimeout() {
//        if (!ENABLE_SMART_TIMEOUTS) {
//            return 5000; // Default 5 second timeout
//        }
//        if (currentShot > 0) {
//            double timeoutRate = (double) timeoutShots / currentShot;
//            if (timeoutRate > 0.3) { // If >30% timeout rate, increase timeout
//                RPM_MONITOR_TIMEOUT_MS += 500;
//            }
//        }
//        return (long) RPM_MONITOR_TIMEOUT_MS;
//    }
//
//    /**
//     * Calculate adaptive speed based on battery voltage
//     * Reduces speed when battery is low to maintain control
//     *
//     * @return Adjusted max power (0.0-1.0)
//     */
//    private double calculateAdaptiveSpeed() {
//        if (!ENABLE_BATTERY_COMPENSATION) {
//            return 0.4; // Default max power
//        }
//        if (currentVoltage < MIN_VOLTAGE_THRESHOLD) {
//            double speedReduction = (MIN_VOLTAGE_THRESHOLD - currentVoltage) * 0.1;
//            return Math.max(0.2, 0.4 - speedReduction); // Never go below 0.2
//        }
//        return 0.4;
//    }
//
//    /**
//     * Log data point for ML training
//     * Saves shot performance data to CSV file
//     *
//     * @param rpmDipped Did RPM dip (shot detected)?
//     * @param timeout Did shot timeout?
//     */
//    private void logDataPoint(boolean rpmDipped, boolean timeout) {
//        if (!ENABLE_DATA_LOGGING) return;
//
//        MLDataPoint data = new MLDataPoint();
//        data.voltage = currentVoltage;
//        data.targetRPM = ShooterSubsystem.targetRPM;
//        data.actualRPM = shooter.getRPM();
//        data.shotDuration = System.currentTimeMillis() - shotStartTime;
//        data.rpmDipped = rpmDipped;
//        data.timeout = timeout;
//
//        // Keep logging relative to legacy twoPos for continuity with prior analysis
//        Pose currentPose = follower.getPose();
//        data.xError = currentPose.getX() - twoPos.getX();
//        data.yError = currentPose.getY() - twoPos.getY();
//        data.distanceToTarget = currentDistanceToTarget;
//        data.usedLimelight = hasValidTarget;
//        data.timestamp = System.currentTimeMillis();
//
//        trainingData.add(data);
//    }
//
//    /**
//     * Save all training data to CSV file on robot controller
//     * File location: /sdcard/FIRST/ml_limelight_data.csv
//     */
//    private void saveTrainingData() {
//        if (!ENABLE_DATA_LOGGING || trainingData.isEmpty()) return;
//
//        try {
//            File file = new File("/sdcard/FIRST/ml_limelight_data.csv");
//            boolean fileExists = file.exists();
//
//            BufferedWriter writer = new BufferedWriter(new FileWriter(file, true));
//
//            // Write header if new file
//            if (!fileExists) {
//                writer.write("voltage,targetRPM,actualRPM,shotDuration,rpmDipped,timeout,xError,yError,distance,usedLimelight,timestamp\n");
//            }
//
//            // Write all data points
//            for (MLDataPoint data : trainingData) {
//                writer.write(data.toString() + "\n");
//            }
//
//            writer.close();
//            multiTelemetry.addLine("✓ ML+Limelight data saved!");
//        } catch (IOException e) {
//            multiTelemetry.addLine("⚠ Failed to save data: " + e.getMessage());
//        }
//    }
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // PATH BUILDING (UPDATED TO YOUR 5-SEGMENT PLAN)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    /**
//     * Build autonomous paths with optional drift correction
//     * Creates paths Start→P1→P2→P3→P4→P5 according to your screenshot
//     */
//    public void buildPaths() {
//        // Apply manual drift correction only to legacy onePos/twoPos (kept for continuity)
//        Pose correctedOnePos = onePos;
//        Pose correctedTwoPos = twoPos;
//
//        if (ENABLE_POSE_CORRECTION) {
//            correctedOnePos = new Pose(
//                    onePos.getX() + X_DRIFT_CORRECTION,
//                    onePos.getY() + Y_DRIFT_CORRECTION,
//                    onePos.getHeading()
//            );
//            correctedTwoPos = new Pose(
//                    twoPos.getX() + X_DRIFT_CORRECTION,
//                    twoPos.getY() + Y_DRIFT_CORRECTION,
//                    twoPos.getHeading()
//            );
//        }
//
//        // Build 5-waypoint poses using current Dashboard values
//        startPose = new Pose(Waypoints.START_X, Waypoints.START_Y, toHeadingRad(Waypoints.START_HDG_DEG));
//        p1Pose    = new Pose(Waypoints.P1_X,    Waypoints.P1_Y,    toHeadingRad(Waypoints.P1_HDG_DEG));
//        p2Pose    = new Pose(Waypoints.P2_X,    Waypoints.P2_Y,    toHeadingRad(Waypoints.P2_HDG_DEG));
//        p3Pose    = new Pose(Waypoints.P3_X,    Waypoints.P3_Y,    toHeadingRad(Waypoints.P3_HDG_DEG));
//        p4Pose    = new Pose(Waypoints.P4_X,    Waypoints.P4_Y,    toHeadingRad(Waypoints.P4_HDG_DEG));
//        p5Pose    = new Pose(Waypoints.P5_X,    Waypoints.P5_Y,    toHeadingRad(Waypoints.P5_HDG_DEG));
//
//        // Path 1: Start → P1 (shoot position)
//        path1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(p1Pose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), p1Pose.getHeading())
//                .build();
//
//        // Path 2: P1 → P2 (face 3-ball row)
//        path2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(p1Pose), new Point(p2Pose)))
//                .setLinearHeadingInterpolation(p1Pose.getHeading(), p2Pose.getHeading())
//                .build();
//
//        // Path 3: P2 → P3 (slow intake sweep)
//        path3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(p2Pose), new Point(p3Pose)))
//                .setLinearHeadingInterpolation(p2Pose.getHeading(), p3Pose.getHeading())
//                .build();
//
//        // Path 4: P3 → P4 (keep intake on)
//        path4 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(p3Pose), new Point(p4Pose)))
//                .setLinearHeadingInterpolation(p3Pose.getHeading(), p4Pose.getHeading())
//                .build();
//
//        // Path 5: P4 → P5 (back to shoot)
//        path5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(p4Pose), new Point(p5Pose)))
//                .setLinearHeadingInterpolation(p4Pose.getHeading(), p5Pose.getHeading())
//                .build();
//
//        // Keep legacy paths (moveOne/moveTwo) conceptually via correctedOnePos/twoPos
//        // so ML logging remains meaningful; not used for motion.
//        // (No-op build for legacy variables to avoid removing previous features)
//        // moveOne/moveTwo removed from motion but comments retained.
//    }
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // SHOOTING HELPERS (UPDATED FOR DETAILED 3-BALL SEQUENCE)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    /**
//     * Start a 3-ball shooting sequence
//     * Robot must have 3 balls preloaded, hold servo engaged
//     *
//     * SEQUENCE PER SHOT:
//     * 1. Retract hold servo (release ball)
//     * 2. Spin up shooter to target RPM
//     * 3. Turn on intake (feeds ball into shooter)
//     * 4. Wait for RPM dip (ball shot)
//     * 5. Immediately close hold servo (predictive timing)
//     * 6. Wait for RPM recovery
//     * 7. Repeat for shots 2 and 3
//     * 8. On shot 3: Engage push servo for 2 seconds
//     *
//     * @param count Number of shots to fire (should be 3)
//     */
//    private void startShots(int count) {
//        shotsTarget = Math.max(0, count);
//        currentShot = 0;
//        shotStep = 0;
//        shotsRunning = shotsTarget > 0;
//
//        // Ensure hold servo is engaged at start (holding all 3 balls)
//        if (shotsRunning) {
//            servos.engageHold();
//        }
//    }
//
//    /**
//     * Update shooting sequence state machine
//     * Call this every loop during shooting
//     *
//     * @return true when all shots complete, false while shooting
//     */
//    private boolean updateShots() {
//        if (!shotsRunning) return true;
//
//        shooter.update();
//
//        switch (shotStep) {
//            case 0: // Initialize shot - Retract hold servo to release ball
//                currentShot++; // Increment shot counter (1, 2, or 3)
//                servos.retractHold(); // Release ball
//                rpmDipped = false;
//                stepStartTime = System.currentTimeMillis();
//                shotStartTime = System.currentTimeMillis();
//                shotStartPose = follower.getPose();
//                targetRPMForShot = ShooterSubsystem.targetRPM;
//
//                // Switch to AprilTag pipeline for shooting accuracy
//                if (limelightConnected) {
//                    limelight.pipelineSwitch(0);
//                }
//
//                multiTelemetry.addLine(String.format("🎯 Shot %d/%d - Hold Retracted", currentShot, shotsTarget));
//                shotStep = 1;
//                break;
//
//            case 1: // Wait brief delay after hold retract
//                if (System.currentTimeMillis() - stepStartTime >= HOLD_RETRACT_DELAY_MS) {
//                    shooter.startShooter(); // Spin up shooter
//                    stepStartTime = System.currentTimeMillis();
//                    shotStep = 2;
//                }
//                break;
//
//            case 2: // Wait for shooter to reach target RPM
//                if (shooter.getRPM() >= targetRPMForShot * RPM_SPINUP_THRESHOLD) {
//                    multiTelemetry.addLine(String.format("✓ Shooter Ready: %.0f RPM", shooter.getRPM()));
//                    stepStartTime = System.currentTimeMillis();
//                    shotStep = 3;
//                }
//                // Timeout protection
//                if (System.currentTimeMillis() - stepStartTime > 3000) {
//                    multiTelemetry.addLine("⚠ Shooter spinup timeout - proceeding anyway");
//                    shotStep = 3;
//                }
//                break;
//
//            case 3: // Wait before starting intake (predictive timing)
//                if (System.currentTimeMillis() - stepStartTime >= INTAKE_START_DELAY_MS) {
//                    intake.start(); // Turn on intake to feed ball
//                    multiTelemetry.addLine("🔄 Intake ON - Feeding ball");
//                    stepStartTime = System.currentTimeMillis();
//                    shotStep = 4;
//                }
//                break;
//
//            case 4: // Monitor for RPM dip (ball entering shooter)
//                double currentRPM = shooter.getRPM();
//
//                // Check if RPM dipped (ball shot)
//                if (currentRPM < targetRPMForShot * RPM_DIP_THRESHOLD) {
//                    rpmDipped = true;
//                    intake.stop(); // Stop intake immediately
//                    servos.engageHold(); // Close hold servo immediately (predictive)
//
//                    multiTelemetry.addLine(String.format("✓ Shot %d FIRED! RPM: %.0f", currentShot, currentRPM));
//                    logDataPoint(true, false);
//                    successfulShots++;
//
//                    stepStartTime = System.currentTimeMillis();
//                    shotStep = 5;
//                }
//
//                // Timeout protection
//                if (System.currentTimeMillis() - stepStartTime > getSmartTimeout()) {
//                    multiTelemetry.addLine(String.format("⚠ Shot %d TIMEOUT", currentShot));
//                    intake.stop();
//                    servos.engageHold();
//                    logDataPoint(false, true);
//                    timeoutShots++;
//                    stepStartTime = System.currentTimeMillis();
//                    shotStep = 5;
//                }
//                break;
//
//            case 5: // Wait for RPM recovery before next shot
//                if (shooter.getRPM() >= targetRPMForShot * RPM_SPINUP_THRESHOLD) {
//                    // Check if this was the 3rd shot
//                    if (currentShot >= shotsTarget) {
//                        // 3rd shot complete - engage push servo
//                        if (currentShot == 3) {
//                            servos.engagePush();
//                            multiTelemetry.addLine("🚀 Push Servo ENGAGED (3rd shot)");
//                        }
//                        stepStartTime = System.currentTimeMillis();
//                        shotStep = 6;
//                    } else {
//                        // More shots remaining - wait recovery delay then next shot
//                        if (System.currentTimeMillis() - stepStartTime >= SHOT_RECOVERY_DELAY_MS) {
//                            multiTelemetry.addLine(String.format("➡ Ready for shot %d", currentShot + 1));
//                            shotStep = 0; // Next shot
//                        }
//                    }
//                }
//                break;
//
//            case 6: // Final cleanup - wait for push servo duration (3rd shot only)
//                if (currentShot == 3) {
//                    if (System.currentTimeMillis() - stepStartTime >= PUSH_SERVO_DURATION_MS) {
//                        servos.retractPush(); // Retract push servo
//                        multiTelemetry.addLine("✓ Push Servo RETRACTED");
//                        shotStep = 7;
//                    }
//                } else {
//                    shotStep = 7; // Skip push servo for non-3rd shots
//                }
//                break;
//
//            case 7: // All shots complete
//                intake.stop();
//                transfer.stop();
//                shooter.stopShooter();
//                shotsRunning = false;
//                multiTelemetry.addLine(String.format("✅ Shooting Complete: %d/%d shots", successfulShots, shotsTarget));
//                break;
//        }
//
//        return !shotsRunning;
//    }
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // AUTONOMOUS STATE MACHINE (UPDATED SEQUENCE)
//    // ═══════════════════════════════════════════════════════════════════════
//
//    /**
//     * Main autonomous update loop
//     * Sequence:
//     * S0: Path1 to shooting position
//     * S1: Shoot PRELOAD_SHOTS (3)
//     * S2: Path2 to face 3-ball row, turn intake ON
//     * S3: Path3 slow sweep, intake ON
//     * S4: Path4 return across, intake ON
//     * S5: Path5 back to shoot, intake OFF
//     * S6: Shoot CYCLE_SHOTS (3)
//     * S7: Complete (you can add parking here)
//     */
//    public void autonomousPathUpdate() {
//        // Update sensors every loop
//        currentVoltage = voltageSensor.getVoltage();
//        updateLimelightData();
//
//        // Periodic AprilTag pose correction
//        updatePoseFromAprilTag();
//
//        // Calculate adaptive values
//        double adaptiveRPM = calculateDistanceBasedRPM(); // Uses Limelight distance if available
//        adaptiveMaxPower = calculateAdaptiveSpeed();
//        ShooterSubsystem.targetRPM = adaptiveRPM;
//
//        switch (pathState) {
//            case 0: // Path1 to shooting spot
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(Waypoints.P1_MAX_POWER);
//                    follower.followPath(path1);
//                }
//                if (!follower.isBusy()) {
//                    startShots(Waypoints.PRELOAD_SHOTS);
//                    setPathState(1);
//                }
//                break;
//
//            case 1: // PRELOAD_SHOTS at Path1 end (3 balls)
//                if (updateShots()) {
//                    setPathState(2);
//                }
//                break;
//
//            case 2: // Path2 to face 3-ball row (turn intake on when leaving)
//                if (!follower.isBusy()) {
//                    intake.start();
//                    transfer.start();
//                    follower.setMaxPower(Waypoints.P2_MAX_POWER);
//                    follower.followPath(path2);
//                }
//                if (!follower.isBusy()) {
//                    setPathState(3);
//                }
//                break;
//
//            case 3: // Path3 slow sweep, intake ON
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(Waypoints.P3_MAX_POWER); // slow
//                    follower.followPath(path3);
//                }
//                if (!follower.isBusy()) {
//                    setPathState(4);
//                }
//                break;
//
//            case 4: // Path4, keep intake ON
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(Waypoints.P4_MAX_POWER);
//                    follower.followPath(path4);
//                }
//                if (!follower.isBusy()) {
//                    setPathState(5);
//                }
//                break;
//
//            case 5: // Path5 back to shooting, then intake OFF and shoot again
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(Waypoints.P5_MAX_POWER);
//                    follower.followPath(path5);
//                }
//                if (!follower.isBusy()) {
//                    intake.stop();
//                    transfer.stop();
//                    startShots(Waypoints.CYCLE_SHOTS);
//                    setPathState(6);
//                }
//                break;
//
//            case 6: // Second shooting cycle (3 balls collected)
//                if (updateShots()) {
//                    setPathState(7);
//                }
//                break;
//
//            case 7: // Done (park logic can be added here if desired)
//                setPathState(-1); // Autonomous complete
//                break;
//        }
//    }
//
//    /**
//     * Set path state and reset timer
//     *
//     * @param pState New path state
//     */
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    // ═══════════════════════════════════════════════════════════════════════
//    // OPMODE LIFECYCLE
//    // ═══════════════════════════════════════════════════════════════════════
//
//    @Override
//    public void init() {
//        // Initialize timers
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        // Set Pedro Pathing constants
//        Constants.setConstants(FConstants.class, LConstants.class);
//
//        // Initialize follower and set starting pose (to your new Start)
//        follower = new Follower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//        // Initialize hardware
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
//        DcMotor feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");
//        Servo pushServo = hardwareMap.get(Servo.class, "push_servo");
//        Servo holdServo = hardwareMap.get(Servo.class, "hold_servo");
//
//        // Initialize telemetry (FTC Dashboard + Driver Station)
//        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Initialize subsystems
//        shooter = new ShooterSubsystem(hardwareMap, multiTelemetry);
//        intake = new IntakeSubsystem(intakeMotor);
//        transfer = new TransferSubsystem(feedMotor);
//        servos = new ServoSubsystem(pushServo);
//        servos.setHoldServo(holdServo);
//
//        // Initialize voltage sensor
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        currentVoltage = voltageSensor.getVoltage();
//
//        // Initialize Limelight
//        initializeLimelight();
//
//        // Display initialization status
//        multiTelemetry.addLine("═══════════════════════════════════");
//        multiTelemetry.addLine("  LIMELIGHT 3A + ML ENHANCED");
//        multiTelemetry.addLine("  DECODE 2025-2026 Season");
//        multiTelemetry.addLine("═══════════════════════════════════");
//        multiTelemetry.addLine();
//
//        multiTelemetry.addLine("🎯 LIMELIGHT FEATURES:");
//        multiTelemetry.addData("  Limelight", limelightConnected ? "✓ CONNECTED" : "✗ NOT FOUND");
//        multiTelemetry.addData("  AprilTag Correction", ENABLE_APRILTAG_CORRECTION ? "✓ ON" : "✗ OFF");
//        multiTelemetry.addData("  Artifact Detection", ENABLE_ARTIFACT_DETECTION ? "✓ ON" : "✗ OFF");
//        multiTelemetry.addData("  Distance Shooting", ENABLE_DISTANCE_SHOOTING ? "✓ ON" : "✗ OFF");
//        multiTelemetry.addLine();
//
//        multiTelemetry.addLine("🧠 ML FEATURES:");
//        multiTelemetry.addData("  Adaptive RPM", ENABLE_ADAPTIVE_RPM ? "✓ ON" : "✗ OFF");
//        multiTelemetry.addData("  Predictive Timing", ENABLE_PREDICTIVE_TIMING ? "✓ ON" : "✗ OFF");
//        multiTelemetry.addData("  Smart Timeouts", ENABLE_SMART_TIMEOUTS ? "✓ ON" : "✗ OFF");
//        multiTelemetry.addData("  Battery Compensation", ENABLE_BATTERY_COMPENSATION ? "✓ ON" : "✗ OFF");
//        multiTelemetry.addLine();
//
//        multiTelemetry.addData("Battery Voltage", "%.2f V", currentVoltage);
//        multiTelemetry.addData("Base RPM", "%.0f", calculateAdaptiveRPM());
//        multiTelemetry.update();
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        // Update subsystems
//        follower.update();
//        shooter.update();
//
//        // Run autonomous state machine
//        autonomousPathUpdate();
//
//        // Display telemetry
//        multiTelemetry.addLine("═══ AUTONOMOUS STATUS ═══");
//        multiTelemetry.addData("Path State", pathState);
//        multiTelemetry.addData("Shot", "%d / %d (Step %d)", currentShot, shotsTarget, shotStep);
//        multiTelemetry.addLine();
//
//        multiTelemetry.addLine("═══ LIMELIGHT VISION ═══");
//        multiTelemetry.addData("Connected", limelightConnected ? "✓ YES" : "✗ NO");
//        multiTelemetry.addData("Valid Target", hasValidTarget ? "✓ YES" : "✗ NO");
//        if (hasValidTarget) {
//            multiTelemetry.addData("Distance", "%.1f in", currentDistanceToTarget);
//            multiTelemetry.addData("Offset (tx)", "%.1f°", targetTx);
//            multiTelemetry.addData("Target Area", "%.1f%%", targetArea);
//        }
//        multiTelemetry.addData("AprilTag Corrections", aprilTagCorrections);
//        multiTelemetry.addData("Green Artifacts", greenArtifactsDetected);
//        multiTelemetry.addData("Purple Artifacts", purpleArtifactsDetected);
//        multiTelemetry.addLine();
//
//        multiTelemetry.addLine("═══ SHOOTER STATUS ═══");
//        multiTelemetry.addData("Target RPM", "%.0f", ShooterSubsystem.targetRPM);
//        multiTelemetry.addData("Actual RPM", "%.1f", shooter.getRPM());
//        multiTelemetry.addData("RPM Source", hasValidTarget ? "Distance-Based" : "Voltage-Based");
//        multiTelemetry.addLine();
//
//        multiTelemetry.addLine("═══ ML METRICS ═══");
//        multiTelemetry.addData("Battery", "%.2f V", currentVoltage);
//        multiTelemetry.addData("Adaptive Speed (Segment)", "%.2f", adaptiveMaxPower);
//        if (currentShot > 0) {
//            multiTelemetry.addData("Success Rate", "%d/%d (%.0f%%)",
//                    successfulShots, currentShot, (double)successfulShots/currentShot*100);
//        }
//        multiTelemetry.addLine();
//
//        multiTelemetry.addLine("═══ POSITION ═══");
//        multiTelemetry.addData("X", "%.2f in", follower.getPose().getX());
//        multiTelemetry.addData("Y", "%.2f in", follower.getPose().getY());
//        multiTelemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
//
//        multiTelemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        // Stop all subsystems
//        intake.stop();
//        transfer.stop();
//        shooter.stopShooter();
//
//        // Stop Limelight
//        if (limelightConnected) {
//            limelight.stop();
//        }
//
//        // Save training data
//        saveTrainingData();
//
//        // Display final statistics
//        multiTelemetry.addLine("═══════════════════════════════════");
//        multiTelemetry.addLine("  FINAL STATISTICS");
//        multiTelemetry.addLine("═══════════════════════════════════");
//        multiTelemetry.addData("Total Shots (last cycle)", currentShot);
//        multiTelemetry.addData("Successful", successfulShots);
//        multiTelemetry.addData("Timeouts", timeoutShots);
//        if (currentShot > 0) {
//            multiTelemetry.addData("Success Rate", "%.1f%%", (double)successfulShots/currentShot*100);
//        }
//        multiTelemetry.addData("AprilTag Corrections", aprilTagCorrections);
//        multiTelemetry.addData("Green Artifacts", greenArtifactsDetected);
//        multiTelemetry.addData("Purple Artifacts", purpleArtifactsDetected);
//        multiTelemetry.addData("Data Points Logged", trainingData.size());
//        multiTelemetry.update();
//    }
//
//}
//
///*
// * ═══════════════════════════════════════════════════════════════════════════
// * DECODE 2025-2026 ARTIFACT COLORS
// * ═══════════════════════════════════════════════════════════════════════════
// *
// * ARTIFACT COLORS:
// * - GREEN: Alliance-specific artifacts
// * - PURPLE: Alliance-specific artifacts
// *
// * LIMELIGHT PIPELINE CONFIGURATION:
// * Pipeline 1: GREEN Artifact Detection
// *   - Hue: 60-90 (green range)
// *   - Saturation: 100-255
// *   - Value: 100-255
// *   - Min Area: 0.5%
// *
// * Pipeline 2: PURPLE Artifact Detection
// *   - Hue: 270-300 (purple/magenta range)
// *   - Saturation: 100-255
// *   - Value: 100-255
// *   - Min Area: 0.5%
// *
// * USAGE EXAMPLE:
// * To find nearest green artifact:
// *   Pose greenArtifact = findNearestArtifact("green");
// *
// * To find nearest purple artifact:
// *   Pose purpleArtifact = findNearestArtifact("purple");
// * ello me llamo steve
// * ═══════════════════════════════════════════════════════════════════════════
// * FIELD COORDINATES (DECODE 2025-2026)
// * ═══════════════════════════════════════════════════════════════════════════
// *
// * ⚠️ IMPORTANT: These coordinates are placeholders!
// * You MUST measure your actual field positions and update (now handled in Waypoints):
// *    - START_X/START_Y/START_HDG_DEG (starting position)
// *    - P1..P5 (path endpoints & headings)
// *
// * SCORING ZONES:
// * - Classifier: High scoring zone (48" height) - requires shooting
// * - Depot: Low scoring zone (12" height) - requires placement
// *
// * NAVIGATION:
// * - Gates: Autonomous navigation zones (bonus points)
// * - Secret Tunnel: Alternative path (bonus points)
// *
// * ═══════════════════════════════════════════════════════════════════════════
// */