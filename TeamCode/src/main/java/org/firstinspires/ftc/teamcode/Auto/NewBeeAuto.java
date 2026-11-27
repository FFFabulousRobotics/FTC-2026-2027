package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
        START, SHOOT_X, SHOOT, GARDEN, INTAKE, SHOOT_Y, END
    }

    public static double INTAKE_TIME = 1.0;
    private final Shoot shoot = new Shoot();
    private final Intake intake = new Intake();
    private final PinpointLocalization localization = new PinpointLocalization();
    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private PathState state = PathState.START;
    private Pose goal;
    private int start = 1;
    private boolean secondShot;

    @Override
    public void init() {
        shoot.init(hardwareMap);
        intake.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        localization.initAuto(hardwareMap, new Pose(0, 0, 0));
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) start = 1;
        if (gamepad1.bWasPressed()) start = 2;
        localization.update();
        telemetry.addLine("A 起点 1，B 起点 2");
        telemetry.addData("Start", start);
        telemetry.addData("Pinpoint ready", localization.isReady());
        telemetry.update();
    }

    @Override
    public void start() {
        goal = start == 1 ? Position.SHOOT_POSE_1 : Position.SHOOT_POSE_2;
        secondShot = false;
        state = PathState.START;
    }

    @Override
    public void loop() {
        if (state != PathState.START) follower.update();
        switch (state) {
            case START:
                localization.update();
                if (localization.isReady()) {
                    follower.setStartingPose(start == 1 ? Position.START_POSE_1 : Position.START_POSE_2);
                    state = PathState.SHOOT_X;
                }
                break;
            case SHOOT_X:
            case SHOOT_Y:
                goal = secondShot
                        ? (start == 1 ? Position.SHOOT_POSE_2 : Position.SHOOT_POSE_1)
                        : (start == 1 ? Position.SHOOT_POSE_1 : Position.SHOOT_POSE_2);
                intake.stop();
                shoot.gateOpen();
                timer.reset();
                state = PathState.SHOOT;
                break;
            case SHOOT:
                if (timer.seconds() >= (Shoot.distance() > 115 ? 0.6 : 0.55)) {
                    intake.stop();
                    shoot.gateClose();
                    if (secondShot) {
                        state = PathState.END;
                        stop();
                    } else {
                        GoToPosition.garden(follower);
                        state = PathState.GARDEN;
                    }
                } else if (timer.seconds() >= 0.2) intake.feed();
                break;
            case GARDEN:
                if (!follower.isBusy()) {
                    intake.intake();
                    timer.reset();
                    state = PathState.INTAKE;
                }
                break;
            case INTAKE:
                if (timer.seconds() >= INTAKE_TIME) {
                    intake.stop();
                    secondShot = true;
                    state = PathState.SHOOT_Y;
                }
                break;
            case END:
                break;
        }
        telemetry.addData("State", state);
        if (state != PathState.START && state != PathState.END) shoot.update(telemetry, goal);
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.stop();
        shoot.stop();
    }
}
