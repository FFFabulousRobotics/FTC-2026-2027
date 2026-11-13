package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Position;

public class Shoot {
    public enum Goal { FIRST, SECOND }

    private double distance, velocity;
    private GoBildaPinpointDriver pp;
    private DcMotorEx flywheel;
    private Goal goal;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    private double getDistance() {
        if (goal == null) return Double.NaN;
        Pose target = goal == Goal.FIRST ? Position.SHOOT_POSE_1 : Position.SHOOT_POSE_2;
        if (target == null) return Double.NaN;

        // follower.update() 已刷新 Pinpoint；目标坐标与位姿均使用英寸。
        Pose2D pose = pp.getPosition();
        double dx = target.getX() - pose.getX(DistanceUnit.INCH);
        double dy = target.getY() - pose.getY(DistanceUnit.INCH);
        return Math.hypot(dx, dy);
    }

    private double Target(double Distance) {
        return Distance + 1;
    }

    public void update() {
        distance = getDistance();
        velocity = Double.isFinite(distance) ? Target(distance) : 0.0;
        flywheel.setVelocity(velocity);
    }
}
