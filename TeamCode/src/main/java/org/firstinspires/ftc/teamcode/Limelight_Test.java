package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*Limelight_Test仅作为测试limelight是否在正常且正确地工作的工具*/
@TeleOp(name = "Limelight (Test)", group = "Test")
public class Limelight_Test extends OpMode {
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
