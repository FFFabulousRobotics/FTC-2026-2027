package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GameMechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class ShabbyAuto extends OpMode {
    private final GameMechanisms mechanisms = new GameMechanisms();
    private Follower follower;

    protected abstract Pose getStartPose();

    @Override
    public void init() {
        mechanisms.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void start() {
        follower.setStartingPose(getStartPose());
        GoToPosition.park(follower);
    }

    @Override
    public void loop() {
        follower.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        mechanisms.stop();
    }
}
