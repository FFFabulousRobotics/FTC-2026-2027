package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Limelight.Limelight;

/*Limelight_Test仅作为测试limelight是否在正常且正确地工作的工具*/
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends OpMode {
    private Limelight vision;

    @Override public void init()  { vision = new Limelight(this); }
    @Override public void start() { vision.start(); }

    @Override public void loop() {
        vision.update();
        telemetry.addData("Tag", vision.getTagId());
        if (vision.ishasTarget()) {
            telemetry.addData("X", vision.getTx());
            telemetry.addData("Y", vision.getTy());
        } else {
            telemetry.addData("状态", "未捕获目标");
        }
        telemetry.update();
    }
}