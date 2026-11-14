package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Net Test", group = "Test")
public class NetTest extends OpMode {
    private Servo net;
    private double commandedPosition;

    @Override
    public void init() {
        net = hardwareMap.get(Servo.class, "net");
        commandedPosition = 0.0;
        net.setPosition(commandedPosition);
        showTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            commandedPosition = commandedPosition == 0.0 ? 1.0 : 0.0;
            net.setPosition(commandedPosition);
        }
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("按 X 切换 net 位置");
        telemetry.addData("net 指令位置", "%.1f", commandedPosition);
        telemetry.update();
    }
}
