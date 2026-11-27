package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Shoot;

@TeleOp(name = "Gate Test", group = "Test")
public class GateTest extends OpMode {
    private ServoImplEx gate;
    private double position;

    @Override
    public void init() {
        gate = hardwareMap.get(ServoImplEx.class, "gate");
        gate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gate.setDirection(Servo.Direction.FORWARD);
        position = Shoot.GATE_CLOSED_POSITION;
        updateGate();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            position = position == Shoot.GATE_OPEN_POSITION
                    ? Shoot.GATE_CLOSED_POSITION : Shoot.GATE_OPEN_POSITION;
        }
        if (gamepad1.aWasPressed()) position = Shoot.GATE_CLOSED_POSITION;
        if (gamepad1.bWasPressed()) position = Shoot.GATE_OPEN_POSITION;
        updateGate();
    }

    private void updateGate() {
        gate.setPosition(position);
        telemetry.addLine("X 切换开闸 / 关闸");
        telemetry.addLine("A 关闸 0.0，B 开闸 0.3，松开保持");
        telemetry.addData("Gate", position == Shoot.GATE_OPEN_POSITION ? "OPEN"
                : position == Shoot.GATE_CLOSED_POSITION ? "CLOSED" : "TEST POSITION");
        telemetry.addData("Gate command", gate.getPosition());
        telemetry.update();
    }
}
