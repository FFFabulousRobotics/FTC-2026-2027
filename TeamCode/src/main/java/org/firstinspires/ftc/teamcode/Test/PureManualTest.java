package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Pure Manual Test", group = "Test")
public class PureManualTest extends OpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void init() {
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
        telemetry.addLine("左摇杆移动，右摇杆左右转向；按住 X 使用 30% 功率，松开 X 全速");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double driveScale = gamepad1.x ? 0.3 : 1.0;
        leftFront.setPower((y + x + rx) / d * driveScale);
        rightFront.setPower((y - x - rx) / d * driveScale);
        leftBack.setPower((y - x + rx) / d * driveScale);
        rightBack.setPower((y + x - rx) / d * driveScale);
        telemetry.addData("Drive scale", driveScale);
        telemetry.addData("lf / lb power", "%.2f / %.2f", leftFront.getPower(), leftBack.getPower());
        telemetry.addData("rf / rb power", "%.2f / %.2f", rightFront.getPower(), rightBack.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
