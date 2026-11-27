package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Auto.Position;

public class Shoot {
    // 沿用去年 HOOD_MIN 对应的背板零位，正式程序和测试共用。
    public static final double BOARD_INITIAL_POSITION = 0.029;
    public static final double GATE_OPEN_POSITION = 0.3;
    public static final double GATE_CLOSED_POSITION = 0.0;
    public static GoBildaPinpointDriver pp;
    private DcMotorEx leftflywheel, rightflywheel;
    private ServoImplEx board;
    private ServoImplEx gate;
    private final Turret turret = new Turret();
    private final MovingShotController movingShotController = new MovingShotController();
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
        board = hardwareMap.get(ServoImplEx.class, "board");
        initBoard(board);
        gate = hardwareMap.get(ServoImplEx.class, "gate");
        gate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gateClose();
        turret.init(hardwareMap);
    }

    public static void initBoard(ServoImplEx board) {
        board.setPwmRange(new PwmControl.PwmRange(500, 2500));
        board.setPosition(BOARD_INITIAL_POSITION);
    }

    public void gateOpen() {
        gate.setPosition(GATE_OPEN_POSITION);
    }

    public void gateClose() {
        gate.setPosition(GATE_CLOSED_POSITION);
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
        update(telemetry, target());
    }

    public void update(Telemetry telemetry, Pose target) {
        goal = target;
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
