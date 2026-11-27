package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Turret;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends OpMode {
    private static final double TURN_SPEED_DEG_PER_SEC = 30.0;
    private final Turret turret = new Turret();
    private final ElapsedTime timer = new ElapsedTime();
    private double angle;

    @Override
    public void init() {
        // 只初始化两个云台舵机，并回到含预紧的正前方中位。
        turret.init(hardwareMap);
        angle = 0;
        showTelemetry();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        double dt = timer.seconds();
        timer.reset();
        int direction = (gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0);
        angle += direction * TURN_SPEED_DEG_PER_SEC * dt;
        angle = Range.clip(angle, Turret.TURRET_MIN_SAFE_ANGLE, Turret.TURRET_MAX_SAFE_ANGLE);
        if (gamepad1.a) angle = 0;
        turret.lockTurretAtAngle(angle);
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("按住方向键左/右转动，松开保持；A 回中位");
        telemetry.addData("Angle command (deg)", "%.1f", angle);
        telemetry.addData("Turn speed (deg/s)", TURN_SPEED_DEG_PER_SEC);
        telemetry.addData("leftturret command", turret.getLastServo1Position());
        telemetry.addData("rightturret command", turret.getLastServo2Position());
        telemetry.update();
    }
}
