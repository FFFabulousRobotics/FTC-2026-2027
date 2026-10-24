package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**出于人道主义精神，我最终决定还是加上一些注释，毕竟我在教程里也写了一些类似的要求，比如加this关键字这些的，我自己不照着做未免有些说不过去
 * 语法部分我相信经过一个月的课程学习，已经能够轻松理解了，因此我仅对一些FTC或者limelight3a专有的变量进行说明*/
public class Limelight{
    private final Limelight3A limelight;
    private final IMU imu;
    private double tx, ty, ta;//相对于摄像头的水平偏角，垂直偏角，面积占比（这项对于能够精确识别坐标的队伍毫无用处，只是我出于人道主义精神，认为有必要说明存在这样一个变量）
    private Pose3D botPose;//场地位姿，用来表示一个东西在 3D 空间里的 位置 + 朝向，用处同ta
    private boolean hasTarget;//hasTarget指是否识别到要识别的apirltag（具体识别哪些在apiltag本地网站中调整）
    private int tagId = -1;//apriltag编号，-1就代表还未识别到
    public Limelight(OpMode opMode){
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                /*如果你的 Hub 不是 Logo 朝上、USB 朝前，就要改这两个枚举，如果啥都不写，就会默认是hub平放，usb朝前这样的猎奇位姿*/
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
    }

    public void start(){
        limelight.start();
    }

    public void update() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
        LLResult r = limelight.getLatestResult();
        hasTarget = (r != null && r.isValid());
        if (hasTarget) {
            tx = r.getTx();
            ty = r.getTy();
            ta = r.getTa();
            botPose = r.getBotpose_MT2();//包含机器人在场地里的 x, y, z 和 yaw/pitch/roll,但这个玩意不是相对位置，而是通过apirltag在场上的固定位置计算出其场地坐标，因此没什么用，本赛季主要是通过识别apriltag来判断蜂巢状态
            List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();//创建一个列表变量，把这一帧画面中识别到的apriltag放入一个列表
            tagId = (tags != null && !tags.isEmpty()) ? tags.get(0).getFiducialId() : -1;//只输出第一个apriltagID,因为我暂时想不到如何在无法保证摄像头每一帧只识别到一个apriltag的情况下输出最恰当的apriltag编号
        }else {
            tagId = -1;
        }
    }

    /**以下是封装类与包内其它程序连接的窗口，出于人道主义精神，不建议用同名变量
     * 对于这种短代码块，合并成一行在我看来比较美观*/
    public double getTx()           { return tx; }
    public double getTy()           { return ty; }
    public double getTa()           { return ta; }
    public Pose3D getBotPose()      { return botPose; }
    public boolean ishasTarget()    { return hasTarget; }
    public int getTagId()           { return tagId; }
    public double getFieldX()       { return botPose == null ? 0 : botPose.getPosition().x; }
    public double getFieldY()       { return botPose == null ? 0 : botPose.getPosition().y; }
    public double getFieldHeading() { return botPose == null ? 0 : botPose.getOrientation().getYaw(); }
}