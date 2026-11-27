package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Shoot;

@TeleOp(name = "Servo Zero Test", group = "Test")
public class ServoZeroTest extends OpMode {
    private ServoImplEx leftturret, rightturret, board;

    @Override
    public void init() {
        leftturret = hardwareMap.get(ServoImplEx.class, "leftturret");
        rightturret = hardwareMap.get(ServoImplEx.class, "rightturret");
        board = hardwareMap.get(ServoImplEx.class, "board");
        for (ServoImplEx servo : new ServoImplEx[]{leftturret, rightturret, board}) {
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
            servo.setDirection(Servo.Direction.FORWARD);
        }
        setZero();
    }

    @Override
    public void loop() {
        setZero();
    }

    private void setZero() {
        // 云台正前方的中位（含预紧），以及当前背板初始位置。
        leftturret.setPosition(0.5035);
        rightturret.setPosition(0.4965);
        board.setPosition(Shoot.BOARD_INITIAL_POSITION);
        telemetry.addData("leftturret command", leftturret.getPosition());
        telemetry.addData("rightturret command", rightturret.getPosition());
        telemetry.addData("board command", board.getPosition());
        telemetry.update();
    }
}
