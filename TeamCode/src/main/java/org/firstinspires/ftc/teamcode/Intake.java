package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotorEx intake;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
    }

    public void intake() {
        intake.setPower(1.0);
    }

    public void stopShoot() {
        intake.setPower(0.0);
    }

    public void kick() {
        intake.setPower(-1.0);
    }
}
