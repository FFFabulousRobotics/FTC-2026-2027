package org.firstinspires.ftc.teamcode.ManualOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manual Mecanum Field Centric", group = "Manual")
public class ManualMecanumFieldCentric extends ManualMecanumDrive {
    @Override
    protected boolean isFieldCentric() {
        return true;
    }
}
