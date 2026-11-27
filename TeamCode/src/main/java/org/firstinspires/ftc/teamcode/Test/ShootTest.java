package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.PinpointLocalization;
import org.firstinspires.ftc.teamcode.Shoot;

@TeleOp(name = "Shoot Test", group = "Test")
public class ShootTest extends OpMode {
    private static final double VELOCITY_STEP = 50.0;
    private static final double BOARD_STEP = 0.01;
    private static final Pose2D ZERO_POSE =
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private final Shoot shoot = new Shoot();
    private ServoImplEx board;
    private ServoImplEx gate;
    private final Intake intake = new Intake();
    private boolean enabled;
    private final PinpointLocalization localization = new PinpointLocalization();
    private GoBildaPinpointDriver pinpoint;
    private double targetVelocity;
    private double boardPosition;

    @Override
    public void init() {
        shoot.initFlywheels(hardwareMap);
        targetVelocity = 0;
        intake.init(hardwareMap);
        enabled = false;
        intake.stop();
        board = hardwareMap.get(ServoImplEx.class, "board");
        boardPosition = Shoot.BOARD_INITIAL_POSITION;
        Shoot.initBoard(board);
        gate = hardwareMap.get(ServoImplEx.class, "gate");
        gate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gate.setPosition(Shoot.GATE_OPEN_POSITION);
        localization.init(hardwareMap);
        pinpoint = localization.getDriver();
        showTelemetry();
    }

    @Override
    public void init_loop() {
        showTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) enabled = !enabled;
        if (gamepad1.yWasPressed()) targetVelocity += VELOCITY_STEP;
        if (gamepad1.aWasPressed()) targetVelocity -= VELOCITY_STEP;
        if (gamepad1.leftBumperWasPressed()) boardPosition -= BOARD_STEP;
        if (gamepad1.rightBumperWasPressed()) boardPosition += BOARD_STEP;
        if (gamepad1.bWasPressed()) pinpoint.setPosition(ZERO_POSE);
        targetVelocity = Range.clip(targetVelocity, 0, Shoot.MAX_TARGET_VELOCITY_TICKS_S);
        boardPosition = Range.clip(boardPosition, 0, 1);
        board.setPosition(boardPosition);

        shoot.setVelocity(enabled ? targetVelocity : 0);
        if (enabled) intake.feed();
        else intake.stop();
        showTelemetry();
    }

    private void showTelemetry() {
        Pose2D pose = localization.update();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        telemetry.addData("target", "%.0f", targetVelocity);
        telemetry.addData("currentvelocity", "%.1f", shoot.getMeasuredVelocity());
        telemetry.addData("board", "%.3f", boardPosition);
        telemetry.addData("x", "%.2f", x);
        telemetry.addData("y", "%.2f", y);
        // B 设置的 Pinpoint 零点到当前位置的直线距离，单位 inch。
        telemetry.addData("distance", "%.2f", Math.hypot(x, y));
        telemetry.update();
    }

    @Override
    public void stop() {
        enabled = false;
        intake.stop();
        shoot.setVelocity(0);
        gate.setPosition(Shoot.GATE_CLOSED_POSITION);
    }
}
