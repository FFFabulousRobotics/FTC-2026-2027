package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Translational PIDF Auto Tuner", group = "Pedro Pathing")
public class TranslationalPIDFAutoTuner extends PedroPIDFAutoTuner {
    @Override protected boolean isHeading() { return false; }
}
