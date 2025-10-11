package pedroPathing.constants;


import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;






        FollowerConstants.leftFrontMotorName = "front_left_motor";
        FollowerConstants.leftRearMotorName = "back_left_motor";
        FollowerConstants.rightFrontMotorName = "front_right_motor";
        FollowerConstants.rightRearMotorName = "back_right_motor";




/*
       FollowerConstants.leftFrontMotorName = "rightFront";
       FollowerConstants.leftRearMotorName = "leftBack";
       FollowerConstants.rightFrontMotorName = "leftFront";
       FollowerConstants.rightRearMotorName = "rightBack";
*/
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;


        FollowerConstants.mass = 10;


        FollowerConstants.xMovement = 91.8304832620753; /// this
        FollowerConstants.yMovement = 77.4318996233015;


        FollowerConstants.forwardZeroPowerAcceleration = -59.00706329948933;
        FollowerConstants.lateralZeroPowerAcceleration = -95.74046809358087;


        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID


        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.8,0.0,0.015,0);//(0.75,0.0,0.01,0)
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.69,0,0.008,0); // Not being used, @see useSecondaryHeadingPID


        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015,0,0.0014,0.6,0);//(0.015,0,0.0014,0.6,0)
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID


        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;


        FollowerConstants.pathEndTimeoutConstraint = 75;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
