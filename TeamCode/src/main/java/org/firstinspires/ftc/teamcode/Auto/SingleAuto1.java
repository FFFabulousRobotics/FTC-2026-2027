package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Intake.intake;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Shoot;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.Judge;



@Autonomous
public class SingleAuto1 extends OpMode {

    public enum State {
        IDLE, TO_GARDEN, TO_FIRST_SHOOT_2, CHECK_FIRST_TARGET,
        TO_FIRST_SHOOT_1, TO_FLOWER, TO_SECOND_SHOOT_1,
        CHECK_SECOND_TARGET, TO_SECOND_SHOOT_2, TO_PARK, DONE
    }

    private Follower follower;
    private Limelight limelight;
    private Judge judge;
    private Shoot shoot;
    private Runnable shutdown;
    private State state = State.IDLE;

    /**
     * 之后在 init() 中创建好实际对象，再调用此方法；这里不做硬件映射。
     * shutdown 负责停止飞轮、吸球/送球机构及相机，保留你自己的机构接口。
     */
    public void bindSubsystems(Follower follower, Limelight limelight,
                               Shoot shoot, Runnable shutdown) {
        this.follower = follower;
        this.limelight = limelight;
        this.judge = new Judge(limelight);
        this.shoot = shoot;
        this.shutdown = shutdown;
    }

    @Override
    public void init() {
        state = State.IDLE;
    }

    @Override
    public void start() {
        follower.setStartingPose(Position.START_POSE_1);
        limelight.start();
        enterState(State.TO_GARDEN);
    }

    @Override
    public void loop() {
        if (state == State.IDLE || state == State.DONE) return;

        follower.update();
        limelight.update();
        shoot.update();
        updateState();

        telemetry.addData("State", state);
        telemetry.update();
    }

    private void updateState() {
        switch (state) {
            case TO_GARDEN:
                if (!follower.isBusy()) {
                    intake();
                    enterState(State.TO_FIRST_SHOOT_2);
                }
                break;
            case TO_FIRST_SHOOT_2:
                if (!follower.isBusy()) enterState(State.CHECK_FIRST_TARGET);
                break;
            case CHECK_FIRST_TARGET:
                if (judge.isAprilTag()) {
                    enterState(State.TO_FLOWER);
                } else {
                    enterState(State.TO_FIRST_SHOOT_1);
                }
                break;
            case TO_FIRST_SHOOT_1:
                if (!follower.isBusy()) enterState(State.TO_FLOWER);
                break;
            case TO_FLOWER:
                if (!follower.isBusy()) {
                    intake();
                    enterState(State.TO_SECOND_SHOOT_1);
                }
                break;
            case TO_SECOND_SHOOT_1:
                if (!follower.isBusy()) enterState(State.CHECK_SECOND_TARGET);
                break;
            case CHECK_SECOND_TARGET:
                if (judge.isAprilTag()) {
                    enterState(State.TO_PARK);
                } else {
                    enterState(State.TO_SECOND_SHOOT_2);
                }
                break;
            case TO_SECOND_SHOOT_2:
                if (!follower.isBusy()) enterState(State.TO_PARK);
                break;
            case TO_PARK:
                if (!follower.isBusy()) stop();
                break;
            default:
                break;
        }
    }

    /** 只在进入状态时下发一次路径，不在每轮 loop() 中重复启动。 */
    private void enterState(State next) {
        state = next;
        switch (next) {
            case TO_GARDEN:
                GoToPosition.garden(follower);
                break;
            case TO_FIRST_SHOOT_2:
            case TO_SECOND_SHOOT_2:
                GoToPosition.shoot2(follower);
                break;
            case TO_FIRST_SHOOT_1:
            case TO_SECOND_SHOOT_1:
                GoToPosition.shoot1(follower);
                break;
            case TO_FLOWER:
                GoToPosition.flower(follower);
                break;
            case TO_PARK:
                GoToPosition.park(follower);
                break;
            default:
                break;
        }
    }

    @Override
    public void stop() {
        if (state == State.DONE) return;
        state = State.DONE;
        if (follower != null) follower.breakFollowing();
        if (shutdown != null) shutdown.run();
    }

}
