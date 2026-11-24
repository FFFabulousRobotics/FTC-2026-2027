package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Auto.Position;

public class Shoot {
    public static GoBildaPinpointDriver pp;
    private DcMotorEx leftflywheel, rightflywheel;
    private Servo board;
    private Servo gate;
    private final Turret turret = new Turret();
    private final MovingShotController movingShotController = new MovingShotController();
    private double gateClosedPosition;
    private double gateOpenPosition;
    public static Pose goal;

    public void init(HardwareMap hardwareMap) {
        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        leftflywheel.setPower(0);
        leftflywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        rightflywheel.setPower(0);
        rightflywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        board = hardwareMap.get(Servo.class, "board");
        gate = hardwareMap.get(Servo.class, "gate");
        gateClose();
        turret.init(hardwareMap);
    }

    public void gateOpen() {
        gate.setPosition(gateOpenPosition);
    }

    public void gateClose() {
        gate.setPosition(gateClosedPosition);
    }

    public void stop() {
        leftflywheel.setPower(0);
        rightflywheel.setPower(0);
        gateClose();
    }

    public static Pose target() {
        if(pp.getPosX(DistanceUnit.INCH) > Position.MIDDLE) goal = Position.SHOOT_POSE_2;
        else goal = Position.SHOOT_POSE_1;
        return goal;
    }

    public static double distance() {
        double CurrentX = pp.getPosX(DistanceUnit.INCH);
        double CurrentY = pp.getPosY(DistanceUnit.INCH);
        double DeltaX = CurrentX - goal.getX();
        double DeltaY = CurrentY - goal.getY();
        return Math.sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
    }
    public double velocity() {
        return velocity(distance());
    }

    private double velocity(double distance) {
        double velocity;
        velocity = distance;
        leftflywheel.setVelocity(velocity);
        rightflywheel.setVelocity(velocity);
        return velocity;
    }

    private double angle(double distance) {
        double angle;
        angle = distance;
        board.setPosition(angle);
        return angle;
    }
    public void update(Telemetry telemetry) {
        target();
        // 外部每轮先 pp.update()，这里统一使用这一次读取的定位数据。
        double robotX = pp.getPosX(DistanceUnit.INCH);
        double robotY = pp.getPosY(DistanceUnit.INCH);
        double heading = pp.getHeading(AngleUnit.DEGREES);
        double omega = pp.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        double originX = Turret.turretX(robotX, heading);
        double originY = Turret.turretY(robotY, heading);
        MovingShotController.Result movingShot = movingShotController.update(
                originX, originY, goal.getX(), goal.getY(),
                pp.getVelX(DistanceUnit.INCH), pp.getVelY(DistanceUnit.INCH));
        Turret.AimResult aim = turret.update(robotX, robotY, heading, omega,
                movingShot.aimTargetX, movingShot.aimTargetY, 0.0);
        double targetVelocity = velocity(movingShot.shootDistance);
        double boardPosition = angle(movingShot.shootDistance);
        telemetry.addData("distance (in)", Math.hypot(goal.getX() - originX, goal.getY() - originY));
        telemetry.addData("shot distance (in)", movingShot.shootDistance);
        telemetry.addData("velocity", targetVelocity);
        telemetry.addData("angle", boardPosition);
        telemetry.addData("moving shot", movingShot.active ? "ON" : "STATIC");
        telemetry.addData("moving velocity (in/s)", "%.1f, %.1f",
                movingShot.usedVelocityX, movingShot.usedVelocityY);
        telemetry.addData("moving correction (deg)", movingShot.headingCorrectionDeg);
        telemetry.addData("turret (deg)", "target %.1f | command %.1f",
                aim.targetTurretAngleDeg, aim.turretAngleDeg);
        telemetry.update();
    }
}
