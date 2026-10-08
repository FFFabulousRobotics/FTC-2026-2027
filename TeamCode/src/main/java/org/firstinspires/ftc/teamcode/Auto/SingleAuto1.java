package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Intake.intake;
import static org.firstinspires.ftc.teamcode.Shoot.shoot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Auto.Position;
import org.firstinspires.ftc.teamcode.Auto.GoToPosition;
import org.firstinspires.ftc.teamcode.Shoot;
import org.firstinspires.ftc.teamcode.Intake;



@Autonomous
public class SingleAuto1 extends LinearOpMode {

    //declare all the hardware
    Follower follower;
    ElapsedTime time = new ElapsedTime();

    public void initialize() {
        //hardwareMap
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Position.START_POSE_1);

    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeInInit()) time.reset();
        if (opModeIsActive()) {
            shoot();
            GoToPosition.garden(follower);
            intake();
            GoToPosition.shoot2(follower);
            shoot();
            GoToPosition.flower(follower);
            intake();
            GoToPosition.shoot1(follower);
            shoot();
            GoToPosition.park(follower);
        }
    }

}
