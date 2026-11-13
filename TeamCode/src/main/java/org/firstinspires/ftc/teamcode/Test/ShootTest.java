package org.firstinspires.ftc.teamcode.Test;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Position;

@TeleOp(name = "Shoot Test", group = "Test")
public class ShootTest extends OpMode {
    private static final double FINE_STEP = 50.0;
    private static final double COARSE_STEP = 200.0;

    private DcMotorEx flywheel;
    private GoBildaPinpointDriver pinpoint;
    private double maxVelocity;
    private double targetVelocity;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        maxVelocity = flywheel.getMotorType().getAchieveableMaxTicksPerSecond();
        targetVelocity = 0;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        showTelemetry(0);
    }

    @Override
    public void init_loop() {
        showTelemetry(0);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) targetVelocity += FINE_STEP;
        if (gamepad1.dpadDownWasPressed()) targetVelocity -= FINE_STEP;
        if (gamepad1.dpadRightWasPressed()) targetVelocity += COARSE_STEP;
        if (gamepad1.dpadLeftWasPressed()) targetVelocity -= COARSE_STEP;
        if (gamepad1.aWasPressed()) targetVelocity = 0;
        targetVelocity = Range.clip(targetVelocity, 0, maxVelocity);

        double commandedVelocity = gamepad1.right_bumper ? targetVelocity : 0;
        flywheel.setVelocity(commandedVelocity);
        showTelemetry(commandedVelocity);
    }

    private void showTelemetry(double commandedVelocity) {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double measuredVelocity = flywheel.getVelocity();
        telemetry.addLine("方向键上下 ±50，左右 ±200；A 清零；按住 RB 运转");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("X (in)", "%.2f", x);
        telemetry.addData("Y (in)", "%.2f", y);
        telemetry.addData("Heading (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
        // Position 中的射击点坐标与 Pinpoint 位姿须使用同一场地坐标系（英寸）。
        showDistance("Distance to SHOOT_POSE_1 (in)", Position.SHOOT_POSE_1, x, y);
        showDistance("Distance to SHOOT_POSE_2 (in)", Position.SHOOT_POSE_2, x, y);
        telemetry.addData("Target (ticks/s)", "%.0f", targetVelocity);
        telemetry.addData("Command (ticks/s)", "%.0f", commandedVelocity);
        telemetry.addData("Measured (ticks/s)", "%.1f", measuredVelocity);
        telemetry.addData("Error (ticks/s)", "%.1f", commandedVelocity - measuredVelocity);
        telemetry.addData("Motor max (ticks/s)", "%.0f", maxVelocity);
        telemetry.update();
    }

    private void showDistance(String label, Pose target, double x, double y) {
        if (target == null) {
            telemetry.addData(label, "未配置");
        } else {
            telemetry.addData(label, "%.2f", Math.hypot(target.getX() - x, target.getY() - y));
        }
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.setPower(0);
    }
}
