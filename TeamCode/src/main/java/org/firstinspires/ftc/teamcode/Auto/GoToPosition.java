package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;



public class GoToPosition {
    public static void shoot1(Follower follower) {
        PathChain shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), Position.SHOOT_POSE_1))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Position.SHOOT_POSE_1.getHeading())
                .build();
        follower.followPath(shoot1);
        while(follower.isBusy()) follower.update();
    }

    public static void shoot2(Follower follower) {
        PathChain shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), Position.SHOOT_POSE_2))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Position.SHOOT_POSE_2.getHeading())
                .build();
        follower.followPath(shoot2);
        while(follower.isBusy()) follower.update();
    }

    public static void flower(Follower follower) {
        PathChain flower =  follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), Position.FLOWER))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Position.FLOWER.getHeading())
                .build();
        follower.followPath(flower);
        while(follower.isBusy()) follower.update();
    }

    public static void garden(Follower follower) {
        PathChain garden =  follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), Position.GARDEN))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Position.GARDEN.getHeading())
                .build();
        follower.followPath(garden);
        while(follower.isBusy()) follower.update();
    }

    public static void park(Follower follower) {
        PathChain park =  follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), Position.PARK))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Position.PARK.getHeading())
                .build();
        follower.followPath(park);
        while(follower.isBusy()) follower.update();
    }
}