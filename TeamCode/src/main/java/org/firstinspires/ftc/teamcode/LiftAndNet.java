package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftAndNet {
    private DcMotor lift;
    private Servo net;
    private boolean netOpen;

    public void init(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotor.class, "lift");
        net = hardwareMap.get(Servo.class, "net");
        stop();
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        netOpen = false;
        net.setPosition(0.0);
    }

    public void update(boolean raise, boolean lower) {
        lift.setPower((lower ? 1.0 : 0.0) - (raise ? 1.0 : 0.0));
    }

    public void toggleNet() {
        netOpen = !netOpen;
        net.setPosition(netOpen ? 1.0 : 0.0);
    }

    public void stop() {
        lift.setPower(0);
    }
}
