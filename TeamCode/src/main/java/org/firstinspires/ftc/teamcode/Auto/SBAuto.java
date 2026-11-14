package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameMechanisms;

@Autonomous(name = "sbauto")
public class SBAuto extends OpMode {
    private static final double DRIVE_POWER = 0.4;
    private static final double DRIVE_SECONDS = 0.5;

    private final ElapsedTime timer = new ElapsedTime();
    private DcMotor[] motors;

    @Override
    public void init() {
        GameMechanisms.initServos(hardwareMap);
        motors = new DcMotor[] {
                initMotor("lf", Direction.REVERSE),
                initMotor("lb", Direction.FORWARD),
                initMotor("rf", Direction.FORWARD),
                initMotor("rb", Direction.REVERSE)
        };
    }

    @Override
    public void start() {
        timer.reset();
        setDrivePower(DRIVE_POWER);
    }

    @Override
    public void loop() {
        if (timer.seconds() >= DRIVE_SECONDS) {
            setDrivePower(0);
            requestOpModeStop();
        }
    }

    @Override
    public void stop() {
        setDrivePower(0);
    }

    private DcMotor initMotor(String name, Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void setDrivePower(double power) {
        for (DcMotor motor : motors) motor.setPower(power);
    }
}
