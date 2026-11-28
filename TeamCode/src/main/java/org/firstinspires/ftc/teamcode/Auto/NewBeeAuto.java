package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.PinpointLocalization;
import org.firstinspires.ftc.teamcode.Shoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class NewBeeAuto extends OpMode {
    public enum PathState {
        START, WAIT_FIRST_SHOT, SHOOT_X, SHOOT, PARK, END
    }

    public static double FIRST_SHOT_WAIT_TIME = 3.0;
    public static double SHOT_VELOCITY = 1500.0;
    public static double PARK_TIMEOUT_S = 10.0;
    private final Shoot shoot = new Shoot();
    private final Intake intake = new Intake();
    private final PinpointLocalization localization = new PinpointLocalization();
    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private PathState state = PathState.START;
    private Pose startPose, goal;

    @Override
    public void init() {
        shoot.init(hardwareMap, false, false);
        shoot.useFixedShotVelocity(SHOT_VELOCITY);
        intake.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        startPose = fromPosition(Position.START_POSE_1);
        goal = Position.SHOOT_POSE_1;
        localization.initAuto(hardwareMap, startPose);
    }

    @Override
    public void init_loop() {
        localization.update();
        if (localization.isReady()) shoot.updateAiming(telemetry, goal);
        telemetry.addLine("固定 START_POSE_1；等待 3s → 固定 1500 发射 → PARK；背板不初始化");
        telemetry.addData("Pinpoint ready", localization.isReady());
        telemetry.update();
    }

    @Override
    public void start() {
        state = PathState.START;
    }

    @Override
    public void loop() {
        if (state == PathState.START || state == PathState.END) localization.update();
        else follower.update();
        switch (state) {
            case START:
                if (localization.isReady()) {
                    follower.setStartingPose(startPose);
                    follower.setPose(startPose);
                    localization.update();
                    intake.stop();
                    shoot.gateClose();
                    timer.reset();
                    state = PathState.WAIT_FIRST_SHOT;
                }
                break;
            case WAIT_FIRST_SHOT:
                // 等待期间继续运行下方的瞄准和飞轮调速，不开门、不送球。
                if (timer.seconds() >= FIRST_SHOT_WAIT_TIME) state = PathState.SHOOT_X;
                break;
            case SHOOT_X:
                intake.stop();
                shoot.gateOpen();
                timer.reset();
                state = PathState.SHOOT;
                break;
            case SHOOT:
                if (timer.seconds() >= (Shoot.distance() > 115 ? 0.6 : 0.55)) {
                    intake.stop();
                    shoot.stop();
                    Pose current = follower.getPose();
                    Pose park = fromPosition(Position.PARK);
                    PathChain path = follower.pathBuilder()
                            .addPath(new BezierLine(current, park))
                            .setLinearHeadingInterpolation(current.getHeading(), park.getHeading())
                            .build();
                    follower.followPath(path, false);
                    timer.reset();
                    state = PathState.PARK;
                } else if (timer.seconds() >= 0.2) intake.feed();
                break;
            case PARK:
                if (!follower.isBusy() || timer.seconds() >= PARK_TIMEOUT_S) {
                    state = PathState.END;
                    stop();
                }
                break;
            case END:
                break;
        }
        telemetry.addData("State", state);
        if (state == PathState.WAIT_FIRST_SHOT || state == PathState.SHOOT_X || state == PathState.SHOOT) {
            shoot.useFixedShotVelocity(SHOT_VELOCITY);
            shoot.update(telemetry, goal);
        } else if (localization.isReady()) {
            // 发射结束后仅继续对准，不重启飞轮，不下发背板位置。
            shoot.updateAiming(telemetry, goal);
        }
        telemetry.update();
    }

    /** 当前 Position 中保存的是实测角度，Pedro 路径使用弧度。 */
    private static Pose fromPosition(Pose pose) {
        return new Pose(pose.getX(), pose.getY(), Math.toRadians(pose.getHeading()));
    }

    @Override
    public void stop() {
        if (follower != null) follower.breakFollowing();
        intake.stop();
        shoot.stop();
        if (localization.isReady()) shoot.saveAimingForTeleOp();
    }
}
