package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotorEx intake, trans;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        trans = hardwareMap.get(DcMotorEx.class, "TRANS");
        stop();
    }

    public void intake() {
        intake.setPower(1.0);
        trans.setPower(0.0);
    }

    public void feed() {
        intake.setPower(1.0);
        trans.setPower(1.0);
    }

    public void stop() {
        intake.setPower(0.0);
        trans.setPower(0.0);
    }

    public void kick() {
        intake.setPower(-1.0);
        trans.setPower(-1.0);
    }
}
