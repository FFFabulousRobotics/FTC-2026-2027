package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pose Test", group = "Test")
public class PoseTest extends OpMode {
    private static final double DRIVE_SCALE = 0.4;
    private static final Pose2D ZERO_POSE =
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private final DcMotor[] motors = new DcMotor[4];
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        String[] names = {"lf", "rf", "lb", "rb"};
        for (int i = 0; i < motors.length; i++) {
            DcMotor motor = hardwareMap.get(DcMotor.class, names[i]);
            motors[i] = motor;
            // 与当前 ManualMecanumDrive 的方向一致。
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void init_loop() {
        updatePoseTelemetry("保持静止，等待 Pinpoint READY");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            pinpoint.setPosition(ZERO_POSE);
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double scale = DRIVE_SCALE / Math.max(1, Math.abs(y) + Math.abs(x) + Math.abs(turn));

        motors[0].setPower((y + x + turn) * scale);
        motors[1].setPower((y - x - turn) * scale);
        motors[2].setPower((y - x + turn) * scale);
        motors[3].setPower((y + x - turn) * scale);

        updatePoseTelemetry("左摇杆移动，右摇杆转向，A 设置零点");
    }

    private void updatePoseTelemetry(String controls) {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        telemetry.addLine(controls);
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotor motor : motors) {
            if (motor != null) motor.setPower(0);
        }
    }
}
