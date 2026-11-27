package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Heading PIDF Auto Tuner", group = "Pedro Pathing")
public class HeadingPIDFAutoTuner extends PedroPIDFAutoTuner {
    @Override protected boolean isHeading() { return true; }
}
