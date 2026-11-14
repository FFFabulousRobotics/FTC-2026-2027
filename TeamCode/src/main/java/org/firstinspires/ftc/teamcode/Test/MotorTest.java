package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends OpMode {

    DcMotorEx push, flywheel;
    @Override
    public void init() {
        push = hardwareMap.get(DcMotorEx.class, "push");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    }

    @Override
    public void loop() {
        push.setPower(1.0);
        flywheel.setPower(1.0);
    }
}
