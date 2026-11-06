package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Auto.Position;

public class Shoot {
    private double distance, velocity;
    private GoBildaPinpointDriver pp;
    private DcMotorEx flywheel;

    public Shoot(GoBildaPinpointDriver pp, DcMotorEx flywheel) {
        this.pp = pp;
        this.flywheel = flywheel;
    }

    private double getDistance() {
        return 0.0;
    }

    private double Target(double Distance) {
        return Distance + 1;
    }

    public void update() {
        distance = getDistance();
        velocity = Target(distance);
        flywheel.setVelocity(velocity);
    }
}
