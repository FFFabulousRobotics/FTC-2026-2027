package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Intake;

@Configurable
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends OpMode {
    public static double INTAKE_X_POWER = Intake.MAX_POWER;
    public static double INTAKE_Y_POWER = -Intake.MAX_POWER;
    public static double TRANS_A_POWER = 1.0;
    public static double TRANS_B_POWER = -1.0;

    private DcMotorEx intake, trans;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        trans = hardwareMap.get(DcMotorEx.class, "TRANS");
        stop();
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trans.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        showTelemetry();
    }

    @Override
    public void loop() {
        double intakePower = gamepad1.x ? INTAKE_X_POWER : gamepad1.y ? INTAKE_Y_POWER : 0.0;
        double transPower = gamepad1.a ? TRANS_A_POWER : gamepad1.b ? TRANS_B_POWER : 0.0;
        intake.setPower(Range.clip(intakePower, -Intake.MAX_POWER, Intake.MAX_POWER));
        trans.setPower(Range.clip(transPower, -1.0, 1.0));
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("按住运行，松开停止；Panels 可实时调整四个按键功率");
        telemetry.addData("INTAKE X / Y", "%.2f / %.2f", INTAKE_X_POWER, INTAKE_Y_POWER);
        telemetry.addData("TRANS A / B", "%.2f / %.2f", TRANS_A_POWER, TRANS_B_POWER);
        telemetry.addData("INTAKE power", "%.2f", intake.getPower());
        telemetry.addData("TRANS power", "%.2f", trans.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
        trans.setPower(0.0);
    }
}
