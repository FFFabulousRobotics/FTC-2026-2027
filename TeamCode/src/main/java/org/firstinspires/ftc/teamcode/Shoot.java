package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Position;

public class Shoot {
    private double distance, velocity;
    private GoBildaPinpointDriver pp;
    private DcMotorEx flywheel;
    private Pose goal;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
    }

    public void target() {
        if(pp.getPosX(DistanceUnit.INCH) > Position.MIDDLE) goal = Position.SHOOT_POSE_2;
        else goal = Position.SHOOT_POSE_1;
    }

    public void distance() {
        double CurrentX = pp.getPosX(DistanceUnit.INCH);
        double CurrentY = pp.getPosY(DistanceUnit.INCH);
        double DeltaX = CurrentX - goal.getX();
        double DeltaY = CurrentY - goal.getY();
        distance = Math.sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
    }
    public void velocity() {

    }

    public void update() {
        target();
        distance();
        velocity();
        flywheel.setVelocity(velocity);
    }
}
