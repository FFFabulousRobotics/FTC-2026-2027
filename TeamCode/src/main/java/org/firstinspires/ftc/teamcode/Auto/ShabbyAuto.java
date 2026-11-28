package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PinpointLocalization;
import org.firstinspires.ftc.teamcode.Shoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** 起点 1 向前，起点 2 向车身右侧走双倍距离；不启动射击或进球机构。 */
@Configurable
@Autonomous(name = "ShabbyAuto", group = "Auto")
public class ShabbyAuto extends OpMode {
    public static double LEAVE_DISTANCE_IN = 6.0;
    public static double MAX_POWER = 0.25;
    public static double MOVE_TIMEOUT_S = 4.0;
    public static double READY_TIMEOUT_S = 3.0;

    private enum State { WAITING, MOVING, DONE, FAILED }

    private final PinpointLocalization localization = new PinpointLocalization();
    private final ElapsedTime timer = new ElapsedTime();
    private final Shoot shoot = new Shoot();
    private Follower follower;
    private int selectedStart = 1;
    private Pose startPose, leavePose;
    private State state = State.WAITING;
    private String status = "等待开始";
    private boolean poseApplied;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.breakFollowing();
        localization.initAuto(hardwareMap, new Pose(0, 0, 0));
        shoot.initAiming(hardwareMap, false);
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) selectedStart = 1;
        if (gamepad1.bWasPressed()) selectedStart = 2;
        localization.update();
        applySelectedStart();
        if (poseApplied && localization.isReady()) shoot.updateAiming(telemetry);
        telemetry.addLine("A 起点 1 向前 / B 起点 2 向车身右侧（双倍距离）");
        telemetry.addData("Start", selectedStart);
        Pose selected = selectedStart == 1 ? Position.START_POSE_1 : Position.START_POSE_2;
        if (selected != null) {
            telemetry.addData("Start (in, deg)", "%.2f, %.2f, %.2f",
                    selected.getX(), selected.getY(), selected.getHeading());
        } else {
            telemetry.addLine("所选 START_POSE 尚未填写");
        }
        telemetry.addData("Leave distance (in)", LEAVE_DISTANCE_IN * (selectedStart == 2 ? 2.0 : 1.0));
        telemetry.addData("Pinpoint ready", localization.isReady());
        telemetry.update();
    }

    @Override
    public void start() {
        Pose selected = selectedStart == 1 ? Position.START_POSE_1 : Position.START_POSE_2;
        if (selected == null) {
            finish(State.FAILED, "所选 START_POSE 尚未填写");
            return;
        }
        // 当前 Position 起点来自实测角度，例如 START_POSE_2 的 -92.48°。
        // 只在本程序入口转为 Pedro 所需的 rad，不改动共享坐标。
        startPose = selectedStartPose();
        poseApplied = false;
        double heading = startPose.getHeading();
        double moveDirection = selectedStart == 2 ? heading - Math.PI / 2.0 : heading;
        double moveDistance = LEAVE_DISTANCE_IN * (selectedStart == 2 ? 2.0 : 1.0);
        leavePose = new Pose(startPose.getX() + moveDistance * Math.cos(moveDirection),
                startPose.getY() + moveDistance * Math.sin(moveDirection), heading);
        follower.setMaxPower(Range.clip(MAX_POWER, 0, 1));
        state = State.WAITING;
        status = "等待 Pinpoint 就绪";
        timer.reset();
    }

    @Override
    public void loop() {
        switch (state) {
            case WAITING:
                localization.update();
                if (localization.isReady()) {
                    follower.setStartingPose(startPose);
                    // 同步 PP 的绝对坐标；避免重复 setStartingPose 时库内旧起点变换影响坐标。
                    follower.setPose(startPose);
                    poseApplied = true;
                    localization.update();
                    PathChain path = follower.pathBuilder()
                            .addPath(new BezierLine(startPose, leavePose))
                            .setConstantHeadingInterpolation(startPose.getHeading())
                            .build();
                    follower.followPath(path, false);
                    timer.reset();
                    state = State.MOVING;
                    status = selectedStart == 2 ? "向右离墙（双倍距离）" : "向前离墙";
                } else if (timer.seconds() >= READY_TIMEOUT_S) {
                    finish(State.FAILED, "Pinpoint 等待超时，已停止");
                }
                break;
            case MOVING:
                if (timer.seconds() >= MOVE_TIMEOUT_S) {
                    finish(State.FAILED, "移动超时，已停止");
                    break;
                }
                follower.update();
                if (!localization.isReady()) {
                    finish(State.FAILED, "Pinpoint 未就绪，已停止");
                } else if (!follower.isBusy()) {
                    Pose actual = follower.getPose();
                    double error = Math.hypot(actual.getX() - leavePose.getX(),
                            actual.getY() - leavePose.getY());
                    if (error <= 1.0) finish(State.DONE, "前进完成，已停止");
                    else finish(State.FAILED, "路径结束但未到点，已停止");
                }
                break;
            case DONE:
                localization.update();
                break;
            case FAILED:
                break;
        }
        if (poseApplied && localization.isReady()) {
            shoot.updateAiming(telemetry);
            Pose2D pose = localization.getDriver().getPosition();
            telemetry.addData("Pose (in, deg)", "%.2f, %.2f, %.1f",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        }
        telemetry.addData("State", state);
        telemetry.addData("Status", status);
        telemetry.addData("Start", selectedStart);
        if (leavePose != null) {
            telemetry.addData("Leave target (in)", "%.2f, %.2f", leavePose.getX(), leavePose.getY());
        }
        telemetry.update();
    }

    private void finish(State result, String message) {
        state = result;
        status = message;
        if (follower != null) follower.breakFollowing();
    }

    private Pose selectedStartPose() {
        Pose selected = selectedStart == 1 ? Position.START_POSE_1 : Position.START_POSE_2;
        return selected == null ? null : new Pose(selected.getX(), selected.getY(),
                Math.toRadians(selected.getHeading()));
    }

    /** INIT 选择改变时才设置一次坐标；不会逐帧覆盖实际定位。 */
    private void applySelectedStart() {
        Pose selected = selectedStartPose();
        if (selected == null || !localization.isReady()) return;
        if (!poseApplied || startPose == null || selected.getX() != startPose.getX()
                || selected.getY() != startPose.getY() || selected.getHeading() != startPose.getHeading()) {
            startPose = selected;
            follower.setStartingPose(startPose);
            follower.setPose(startPose);
            poseApplied = true;
            localization.update();
        }
    }

    @Override
    public void stop() {
        finish(State.DONE, "已停止");
        if (poseApplied) shoot.saveAimingForTeleOp();
    }
}
