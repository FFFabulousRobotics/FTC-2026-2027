package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shoot Power Test", group = "Test")
public class ShootPowerTest extends OpMode {
    private DcMotorEx intake, trans, leftflywheel, rightflywheel;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        trans = hardwareMap.get(DcMotorEx.class, "TRANS");
        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        setPower(0);
        for (DcMotorEx motor : new DcMotorEx[]{intake, trans, leftflywheel, rightflywheel}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addLine("START 后四个电机全部 +1，STOP 后停止");
        telemetry.update();
    }

    @Override
    public void loop() {
        setPower(1);
        telemetry.addData("INTAKE / TRANS power", "+1 / +1");
        telemetry.addData("Flywheel power (left / right)", "+1 / +1");
        telemetry.addData("Left velocity (ticks/s)", leftflywheel.getVelocity());
        telemetry.addData("Right velocity (ticks/s)", rightflywheel.getVelocity());
        telemetry.update();
    }

    private void setPower(double power) {
        intake.setPower(power);
        trans.setPower(power);
        leftflywheel.setPower(power);
        rightflywheel.setPower(power);
    }

    @Override
    public void stop() {
        setPower(0);
    }
}
