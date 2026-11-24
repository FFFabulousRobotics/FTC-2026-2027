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
import org.firstinspires.ftc.teamcode.PinpointLocalization;
import org.firstinspires.ftc.teamcode.Turret;

@TeleOp(name = "Shoot Test", group = "Test")
public class ShootTest extends OpMode {
    private static final double FINE_STEP = 50.0;
    private static final double COARSE_STEP = 200.0;

    private DcMotorEx leftflywheel, rightflywheel;
    private final PinpointLocalization localization = new PinpointLocalization();
    private GoBildaPinpointDriver pinpoint;
    private double maxVelocity;
    private double targetVelocity;

    @Override
    public void init() {
        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        leftflywheel.setPower(0);
        rightflywheel.setPower(0);
        leftflywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightflywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        maxVelocity = Math.min(leftflywheel.getMotorType().getAchieveableMaxTicksPerSecond(),
                rightflywheel.getMotorType().getAchieveableMaxTicksPerSecond());
        targetVelocity = 0;
        localization.init(hardwareMap);
        pinpoint = localization.getDriver();
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
        leftflywheel.setVelocity(commandedVelocity);
        rightflywheel.setVelocity(commandedVelocity);
        showTelemetry(commandedVelocity);
    }

    private void showTelemetry(double commandedVelocity) {
        Pose2D pose = localization.update();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.DEGREES);
        double originX = Turret.turretX(x, heading);
        double originY = Turret.turretY(y, heading);
        double leftVelocity = leftflywheel.getVelocity();
        double rightVelocity = rightflywheel.getVelocity();
        telemetry.addLine("方向键上下 ±50，左右 ±200；A 清零；按住 RB 运转");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("X (in)", "%.2f", x);
        telemetry.addData("Y (in)", "%.2f", y);
        telemetry.addData("Heading (deg)", "%.2f", heading);
        telemetry.addData("Turret pivot (in)", "%.2f, %.2f", originX, originY);
        // Position 中的射击点坐标与 Pinpoint 位姿须使用同一场地坐标系（英寸）。
        showDistance("Distance to SHOOT_POSE_1 (in)", Position.SHOOT_POSE_1, originX, originY);
        showDistance("Distance to SHOOT_POSE_2 (in)", Position.SHOOT_POSE_2, originX, originY);
        telemetry.addData("Target (ticks/s)", "%.0f", targetVelocity);
        telemetry.addData("Command (ticks/s)", "%.0f", commandedVelocity);
        telemetry.addData("Left measured (ticks/s)", "%.1f", leftVelocity);
        telemetry.addData("Right measured (ticks/s)", "%.1f", rightVelocity);
        telemetry.addData("Left error (ticks/s)", "%.1f", commandedVelocity - leftVelocity);
        telemetry.addData("Right error (ticks/s)", "%.1f", commandedVelocity - rightVelocity);
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
        leftflywheel.setPower(0);
        rightflywheel.setPower(0);
    }
}
