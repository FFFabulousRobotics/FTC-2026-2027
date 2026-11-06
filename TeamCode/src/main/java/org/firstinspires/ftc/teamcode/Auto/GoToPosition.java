package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;



/** 只下发路径，不等待到达；由 OpMode.loop() 持续更新 follower。 */
public class GoToPosition {
    public static void shoot1(Follower follower) {
        moveTo(follower, Position.SHOOT_POSE_1);
    }

    public static void shoot2(Follower follower) {
        moveTo(follower, Position.SHOOT_POSE_2);
    }

    public static void flower(Follower follower) {
        moveTo(follower, Position.FLOWER);
    }

    public static void garden(Follower follower) {
        moveTo(follower, Position.GARDEN);
    }

    public static void park(Follower follower) {
        moveTo(follower, Position.PARK);
    }

    private static void moveTo(Follower follower, Pose destination) {
        Pose start = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, destination))
                .setLinearHeadingInterpolation(start.getHeading(), destination.getHeading())
                .build();
        follower.followPath(path);
    }
}
