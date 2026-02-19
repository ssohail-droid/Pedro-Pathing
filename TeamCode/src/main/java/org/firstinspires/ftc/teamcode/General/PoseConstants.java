package org.firstinspires.ftc.teamcode.General;

import com.pedropathing.geometry.Pose;

public class PoseConstants {
    //blue side
    private final Pose startPos = SharedData.side == Side.BLUE ? new Pose(32, 135, Math.toRadians(180)) : new Pose(111, 135, Math.toRadians(0));
    private final Pose shootPos = SharedData.side == Side.BLUE ? new Pose(41, 111.05, Math.toRadians(140)) : new Pose(105.331136738056, 108.4, Math.toRadians(40));

    private final Pose intakeRowOnePos = SharedData.side == Side.BLUE ? new Pose(52, 78, Math.toRadians(0)) : new Pose(95, 93, Math.toRadians(180));
    private final Pose intakePickUpRowOnePos = SharedData.side == Side.BLUE ? new Pose(16, 78, Math.toRadians(0)) : new Pose(125.6, 93, Math.toRadians(180));

    private final Pose openGate = SharedData.side == Side.BLUE ? new Pose(17.5, 71, Math.toRadians(0)) : new Pose(127.02337075207845, 90, Math.toRadians(180));
    private final Pose openGateControlPoint = SharedData.side == Side.BLUE ? new Pose(35, 79, Math.toRadians(0)) : new Pose(100, 80);

    private final Pose intakeRowTwoPos = SharedData.side == Side.BLUE ? new Pose(54, 52, Math.toRadians(0)) : new Pose(95, 70, Math.toRadians(180));
    private final Pose intakePickUpRowTwoPos = SharedData.side == Side.BLUE ? new Pose(7, 52, Math.toRadians(0)) : new Pose(131.8, 67, Math.toRadians(180));

    private final Pose leave = SharedData.side == Side.BLUE ? new Pose(50, 123.76, Math.toRadians(180)) : new Pose(96, 126.53915776241358, Math.toRadians(0));


}
