package org.firstinspires.ftc.teamcode.pedroPathing.AutoClass.EnumeratedState.refactored;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


//@Autonomous(name = "EnumeratedStateAuto(V3.0)", group = "Examples")
public class EnumeratedStateAuto extends OpMode {

    private Follower follower;
    private AutoStateMachine autoMachine;
    private Timer opmodeTimer;

    @Override
    public void init() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        autoMachine = new AutoStateMachine(follower);
    }

    @Override
    public void loop() {
        follower.update();
        autoMachine.update();

        telemetry.addData("Path State", autoMachine.getState());
        telemetry.addData("X", autoMachine.getPose().getX());
        telemetry.addData("Y", autoMachine.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(autoMachine.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        autoMachine.setState(State.STATE_START);
    }

    @Override
    public void stop() {}
}
