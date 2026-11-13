package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends OpMode {

    Servo intake;
    @Override
    public void init() {
        intake = hardwareMap.get(Servo.class, "net");
        intake.setPosition(1.0);
    }

    @Override
    public void loop() {
        intake.setPosition(0.5);
    }
}

//intake zero pose -> 1 , max = 0.5
