package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ShabbyAuto1")
public class ShabbyAuto1 extends ShabbyAuto {
    @Override
    protected Pose getStartPose() {
        return Position.START_POSE_1;
    }
}
