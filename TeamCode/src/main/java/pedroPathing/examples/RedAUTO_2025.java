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
//
//import pedroPathing.SubSystem.IntakeSubsystem;
//import pedroPathing.SubSystem.ServoSubsystem;
//import pedroPathing.SubSystem.ShooterSubsystem;
//import pedroPathing.SubSystem.TransferSubsystem;
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@Config
//@Autonomous(name = "RedAUTO 2025 DECODE", group = "SeasonAutos")
//public class RedAUTO_2025 extends BlueAUTO_2025 {
//
//    // Field size for mirror
//    public static final double FIELD_Y_MIRROR = 144.0;
//
//    @Override
//    public void init() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//
//        // mirror Blue poses across Y = 144
//        Pose startPose         = new Pose(22, FIELD_Y_MIRROR - 122.5, Math.toRadians(37));
//        Pose shootPose         = new Pose(52, FIELD_Y_MIRROR - 102.5, Math.toRadians(40));
//        Pose preIntakePose     = new Pose(52, FIELD_Y_MIRROR - 84,   Math.toRadians(180));
//        Pose intakeMovePose    = new Pose(25, FIELD_Y_MIRROR - 84,   Math.toRadians(180));
//        Pose intakeReturnPose  = new Pose(52, FIELD_Y_MIRROR - 84,   Math.toRadians(180));
//        Pose intakeToShootPose = new Pose(52, FIELD_Y_MIRROR - 102.5,Math.toRadians(40));
//
//        follower.setStartingPose(startPose);
//
//        // Re-initialize hardware like Blue
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
//        DcMotor feedMotor   = hardwareMap.get(DcMotor.class, "feed_motor");
//        Servo pushServo     = hardwareMap.get(Servo.class, "push_servo");
//        Servo holdServo     = hardwareMap.get(Servo.class, "hold_servo");
//
//        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        shooter  = new ShooterSubsystem(hardwareMap, multiTelemetry);
//        intake   = new IntakeSubsystem(intakeMotor);
//        transfer = new TransferSubsystem(feedMotor);
//        servos   = new ServoSubsystem(pushServo);
//        servos.setHoldServo(holdServo);
//
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        currentVoltage = voltageSensor.getVoltage();
//
//        // Build mirrored paths
//        toShootFromStart = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(shootPose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
//                .build();
//        toPreIntake = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(shootPose), new Point(preIntakePose)))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), preIntakePose.getHeading())
//                .build();
//        intakeOnMove = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(preIntakePose), new Point(intakeMovePose)))
//                .setLinearHeadingInterpolation(preIntakePose.getHeading(), intakeMovePose.getHeading())
//                .build();
//        intakeOffReturn = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(intakeMovePose), new Point(intakeReturnPose)))
//                .setLinearHeadingInterpolation(intakeMovePose.getHeading(), intakeReturnPose.getHeading())
//                .build();
//        toShootReturn = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(intakeReturnPose), new Point(intakeToShootPose)))
//                .setLinearHeadingInterpolation(intakeReturnPose.getHeading(), intakeToShootPose.getHeading())
//                .build();
//
//        pathTimer = new Timer();
//        multiTelemetry.addLine("✅ RedAUTO_2025 Ready");
//        multiTelemetry.addData("Start Pose", startPose);
//        multiTelemetry.update();
//    }
//}
