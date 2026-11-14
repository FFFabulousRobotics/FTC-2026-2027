package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ShabbyAuto2")
public class ShabbyAuto2 extends ShabbyAuto {
    @Override
    protected Pose getStartPose() {
        return Position.START_POSE_2;
    }
}
