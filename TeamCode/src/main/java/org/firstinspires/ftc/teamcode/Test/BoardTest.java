package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Shoot;

@TeleOp(name = "Board Test", group = "Test")
public class BoardTest extends OpMode {
    private static final double POSITION_STEP = 0.01;
    private ServoImplEx board;
    private double position;

    @Override
    public void init() {
        board = hardwareMap.get(ServoImplEx.class, "board");
        Shoot.initBoard(board);
        position = Shoot.BOARD_INITIAL_POSITION;
        showTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) position += POSITION_STEP;
        if (gamepad1.aWasPressed()) position -= POSITION_STEP;
        position = Range.clip(position, 0.0, 1.0);
        board.setPosition(position);
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("Y（上）位置 +0.01，A（下）位置 -0.01，松开保持");
        telemetry.addData("Board command (0..1)", "%.3f", position);
        telemetry.update();
    }
}
