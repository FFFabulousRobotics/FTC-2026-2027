package org.firstinspires.ftc.teamcode.ManualOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Shoot;

/** 无头驾驶；飞轮和背板人工设置，云台舵机锁定 0.5。 */
@TeleOp(name = "Static Manual", group = "Manual")
public class StaticManual extends ManualOpMode {
    private static final double INITIAL_FLYWHEEL_VELOCITY = 1500.0;
    private static final double VELOCITY_STEP = 20.0;
    private static final double BOARD_STEP = 0.2;

    private double flywheelVelocity = INITIAL_FLYWHEEL_VELOCITY;
    private double boardPosition;

    @Override
    protected boolean isFieldCentric() {
        return true;
    }

    @Override
    protected double drivePowerScale() {
        return gamepad1.x ? 0.5 : 1.0;
    }

    @Override
    protected String turretControlHint() {
        return "云台固定 0.5 / 0.5；按住 X 底盘半速，松开全速";
    }

    @Override
    protected void initShooter(Shoot shooter) {
        shooter.initStatic(hardwareMap);
        boardPosition = shooter.getBoardPosition();
    }

    @Override
    protected void prepareShooter(Shoot shooter, boolean localizationReady) {
        shooter.useFixedShotVelocity(flywheelVelocity);
        telemetry.addLine("固定射击：↑/↓ 转速 ±20；→/← 背板 ±0.2（每按一次）");
        telemetry.addLine("云台锁定 0.5 / 0.5；按住 X 底盘半速；背板不初始化");
        telemetry.addData("static shot", "%.0f ticks/s | board %.3f",
                flywheelVelocity, boardPosition);
    }

    @Override
    protected void updateShooter(Shoot shooter) {
        int velocitySteps = (gamepad1.dpadUpWasPressed() ? 1 : 0)
                - (gamepad1.dpadDownWasPressed() ? 1 : 0);
        int boardSteps = (gamepad1.dpadRightWasPressed() ? 1 : 0)
                - (gamepad1.dpadLeftWasPressed() ? 1 : 0);
        flywheelVelocity = Range.clip(flywheelVelocity + VELOCITY_STEP * velocitySteps,
                0, Shoot.MAX_TARGET_VELOCITY_TICKS_S);
        boardPosition = Range.clip(boardPosition + BOARD_STEP * boardSteps, 0, 1);
        shooter.useFixedShotVelocity(flywheelVelocity);
        if (boardSteps != 0) shooter.useFixedShotSettings(flywheelVelocity, boardPosition);
        shooter.update(telemetry, false, false);
    }
}
