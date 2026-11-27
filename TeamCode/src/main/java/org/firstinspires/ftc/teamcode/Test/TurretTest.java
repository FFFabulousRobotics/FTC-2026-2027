package org.firstinspires.ftc.teamcode.Test;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Auto.Position;
import org.firstinspires.ftc.teamcode.PinpointLocalization;
import org.firstinspires.ftc.teamcode.Turret;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends OpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private final Turret turret = new Turret();
    private final PinpointLocalization localization = new PinpointLocalization();
    private GoBildaPinpointDriver pinpoint;
    private Pose2D pose;
    private Pose goal;
    private Turret.AimResult aim;
    private int selectedTarget; // 0 自动分区，1/2 指定射击框。
    private int targetIndex;

    @Override
    public void init() {
        // 底盘手操、云台自动瞄准；INIT 时放在 START_POSE_1 并保持静止。
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        turret.init(hardwareMap);
        localization.initAuto(hardwareMap, Position.START_POSE_1);
        pinpoint = localization.getDriver();
        selectedTarget = 0;
        aim = null;
        pose = localization.update();
        selectGoal();
        showTelemetry();
    }

    @Override
    public void init_loop() {
        pose = localization.update();
        selectGoal();
        showTelemetry();
    }

    @Override
    public void loop() {
        pose = localization.update();
        selectGoal();
        if (localization.isReady()) drive();
        else stop();
        if (!localization.isReady() || gamepad1.y) {
            turret.center();
            aim = null;
        } else {
            aim = turret.update(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES),
                    pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES),
                    goal.getX(), goal.getY(), 0.0);
        }
        showTelemetry();
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double scale = gamepad1.left_bumper ? 0.3 : 1.0;
        leftFront.setPower((y + x + rx) / d * scale);
        rightFront.setPower((y - x - rx) / d * scale);
        leftBack.setPower((y - x + rx) / d * scale);
        rightBack.setPower((y + x - rx) / d * scale);
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void selectGoal() {
        if (gamepad1.aWasPressed()) selectedTarget = 1;
        if (gamepad1.bWasPressed()) selectedTarget = 2;
        if (gamepad1.xWasPressed()) selectedTarget = 0;
        targetIndex = selectedTarget == 0
                ? (pose.getX(DistanceUnit.INCH) > Position.MIDDLE ? 2 : 1) : selectedTarget;
        goal = targetIndex == 1 ? Position.SHOOT_POSE_1 : Position.SHOOT_POSE_2;
    }

    private void showTelemetry() {
        telemetry.addLine("INIT 放在 START_POSE_1 并保持静止；START 后自动对准");
        telemetry.addLine("左摇杆移动，右摇杆转向；按住 L1 使用 30% 底盘功率");
        telemetry.addLine("A 目标1，B 目标2，X 自动选框；按住 Y 回中，松开恢复瞄准");
        telemetry.addData("Pinpoint", localization.isReady() ? "READY" : "校准中/未就绪");
        telemetry.addData("Target", "%s / SHOOT_POSE_%d (%.2f, %.2f)",
                selectedTarget == 0 ? "AUTO" : "指定", targetIndex, goal.getX(), goal.getY());
        telemetry.addData("Pose (in, deg)", "%.2f, %.2f, %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        if (aim != null) {
            telemetry.addData("Aim", aim.hasTarget ? "TRACKING" : "无有效目标角度");
            telemetry.addData("Target / command (deg)", "%.1f / %.1f",
                    aim.targetTurretAngleDeg, aim.turretAngleDeg);
            telemetry.addData("Turret distance (in)", "%.2f", aim.distance);
        } else telemetry.addData("Aim", gamepad1.y ? "回中" : "等待 START / Pinpoint 就绪");
        telemetry.addData("leftturret command", turret.getLastServo1Position());
        telemetry.addData("rightturret command", turret.getLastServo2Position());
        telemetry.update();
    }
}
