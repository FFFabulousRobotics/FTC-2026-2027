package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
        // 直接设置舵机位置 0.0，不经过云台角度映射或背板曲线。
        leftturret.setPosition(0.0);
        rightturret.setPosition(0.0);
        board.setPosition(0.0);
        telemetry.addData("leftturret command", leftturret.getPosition());
        telemetry.addData("rightturret command", rightturret.getPosition());
        telemetry.addData("board command", board.getPosition());
        telemetry.update();
    }
}
