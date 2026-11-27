package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Auto.BallTarget;
import org.firstinspires.ftc.teamcode.Auto.BallTracker;

/**出于人道主义精神，我最终决定还是加上一些注释，毕竟我在教程里也写了一些类似的要求，比如加this关键字这些的，我自己不照着做未免有些说不过去
 * 语法部分我相信经过一个月的课程学习，已经能够轻松理解了，因此我仅对一些FTC或者limelight3a专有的变量进行说明*/
public class Limelight{
    private final Limelight3A limelight;
    private final IMU imu;
    private double tx, ty, ta;//相对于摄像头的水平偏角，垂直偏角，面积占比（这项对于能够精确识别坐标的队伍毫无用处，只是我出于人道主义精神，认为有必要说明存在这样一个变量）
    private Pose3D botPose;//场地位姿，用来表示一个东西在 3D 空间里的 位置 + 朝向，用处同ta
    private boolean hasTarget;//hasTarget指是否识别到要识别的apirltag（具体识别哪些在apiltag本地网站中调整）
    private int tagId = -1;//apriltag编号，-1就代表还未识别到
    private final List<Integer> tagIds = new ArrayList<>();//这一帧视野里所有的apriltag编号，可能不止一个。只读，别从外面改

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

    public void stop(){
        limelight.stop();
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
        /*tagIds放在if/else外面填：没目标的时候要清空，不能留着上一帧的*/
        tagIds.clear();
        if (hasTarget && r.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult f : r.getFiducialResults()) tagIds.add(f.getFiducialId());
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
    public List<Integer> getTagIds() { return tagIds; }
    public double getFieldX()       { return botPose == null ? 0 : botPose.getPosition().x; }
    public double getFieldY()       { return botPose == null ? 0 : botPose.getPosition().y; }
    public double getFieldHeading() { return botPose == null ? 0 : botPose.getOrientation().getYaw(); }

    /*================= 以下接吃球算法(pipeline 3,自训练模型,标签red/yellow/blue) =================*/

    public static final int PIPE_APRILTAG = 0;//AprilTag管线,蜂巢判断等用
    public static final int PIPE_BALLS    = 3;//自训练检测模型所在管线

    /*两个"安装符号"。各检查一次,不对就只改这里,别动下面的公式:
     * CAM_TX_SIGN : 把球举在车头正右方,telemetry 里 tx 应该是正的
     * IMU_YAW_SIGN: 让车头向左转,yaw 应该变大*/
    private static final double CAM_TX_SIGN  = +1;
    private static final double IMU_YAW_SIGN = +1;

    /*模型类别号 = 训练集 classes.txt 的行号(从0数)。SDK 读不到类别名时,按这张表认颜色*/
    private static final int ID_RED = 0, ID_YELLOW = 1, ID_BLUE = 2;

    /*距离标定常数:d = K/√ta(设计稿§1.1),两种球直径不同各标各的,标定工具 Test/BallKCalib*/
    public static double K_BIG    = 60;//TODO: 标定后替换——大球
    public static double K_POLLEN = 45;//TODO: 标定后替换——花粉

    private boolean allianceRed = false;//红蓝联盟,决定red/blue标签谁是敌人

    private final List<BallTarget> ballFrame = new ArrayList<>();//最近一帧的检测
    private String rawText = "";//最近一帧的原始检测(类别名/类别号/置信度...),调试用
    private boolean framePending = false;//pollFrame 拿到新帧时置位,getBallTargets 读走后清零
    private double lastFrameTs = Double.NaN;//相机侧帧时间戳:同一帧的 ts 不变,靠它判断相机出没出新帧
    private long fpsWindowStartMs;//帧率统计
    private int fpsFrames;
    private double fps;

    public void setAllianceRed(boolean red) { allianceRed = red; }

    /**切换管线,返回是否成功。这是阻塞的HTTP请求(正常几毫秒;相机连着却不应答时最长卡15秒),
     * 所以只在 init 里调一次,千万别放进 loop*/
    public boolean usePipeline(int index) { return limelight.pipelineSwitch(index); }

    public boolean isConnected() { return limelight.isConnected(); }
    public double getFps()       { return fps; }

    /**机器人朝向(度,逆时针为正),已乘上 IMU_YAW_SIGN*/
    public double getYawDeg()    { return IMU_YAW_SIGN * imu.getRobotYawPitchRollAngles().getYaw(); }

    /**轮询相机:出了新的一帧返回 true,这一帧的内容用 getBallTargets() 取;没出新帧返回 false。
     * 没出新帧就什么都别做——同一帧处理两遍,跟踪器的帧数会虚高。
     *
     * 为什么不用 SDK 的 getStaleness():在 SDK 11.1.0 的源码里,它量的是"控制Hub多久没解析到新的HTTP应答",
     * 而Hub每10ms就轮询一次,所以只要网线通着它就几乎是0,哪怕相机卡死在一张旧画面上。
     * 真正能说明"是不是新画面"的是相机自己的帧时间戳 ts。
     * @param yawDeg 机器人此刻的朝向(getYawDeg())。由调用方传进来而不是这里自己读:
     *               读IMU走I²C不便宜,每个循环读一次就够*/
    public boolean pollFrame(double yawDeg) {
        LLResult r = limelight.getLatestResult();
        double ts = r.getTimestamp();
        if (ts == lastFrameTs) return false;
        lastFrameTs = ts;
        countFps();

        ballFrame.clear();
        rawText = "";
        if (r.isValid()) {
            List<LLResultTypes.DetectorResult> boxes = r.getDetectorResults();
            if (boxes != null) {
                int shown = 0;
                for (LLResultTypes.DetectorResult b : boxes) {
                    if (shown++ < 6) rawText += String.format(Locale.ROOT, "[%s|id%d|c%.0f|tx%.1f|ta%.1f] ",
                            b.getClassName(), b.getClassId(), b.getConfidence(), b.getTargetXDegrees(), b.getTargetArea());
                    BallTarget.Kind kind = mapKind(b.getClassName(), b.getClassId());
                    if (kind == null) continue;//不认识的标签:忽略(宁漏勿错)
                    double conf = b.getConfidence();//SDK文档里检测器的置信度是0-100,这里统一成0-1
                    if (conf > 1.0) conf /= 100.0;
                    double txDeg = CAM_TX_SIGN * b.getTargetXDegrees();
                    double ta = b.getTargetArea();
                    double k = (kind == BallTarget.Kind.POLLEN) ? K_POLLEN : K_BIG;
                    ballFrame.add(new BallTarget(kind, ballFrame.size(), txDeg, BallTracker.bearingOf(yawDeg, txDeg), ta,
                            k / Math.sqrt(Math.max(ta, 1e-6)), Math.max(0.0, Math.min(1.0, conf))));
                }
            }
        }
        framePending = true;
        return true;
    }

    /**取 pollFrame() 刚拿到的那一帧。每一帧只能读一次:之后没出新帧就再调,拿到的是空表。
     * 返回的是内部那份列表,下一次 pollFrame() 会把它清掉重用——想留着就自己复制一份(new ArrayList<>(...))*/
    public List<BallTarget> getBallTargets() {
        if (!framePending) return Collections.emptyList();
        framePending = false;
        return ballFrame;
    }

    /**最近一帧的原始检测文本(没经过标签过滤),给 Test/BallKCalib 的telemetry用:
     * 上车第一件事就是看它——确认类别名/类别号到底哪个有值、和红黄蓝对不对得上*/
    public String getRawText() { return rawText; }

    private void countFps() {
        long nowMs = System.currentTimeMillis();
        if (nowMs - fpsWindowStartMs >= 1000) {
            if (fpsWindowStartMs != 0) fps = fpsFrames * 1000.0 / (nowMs - fpsWindowStartMs);
            fpsWindowStartMs = nowMs;
            fpsFrames = 0;
        }
        fpsFrames++;
    }

    /**标签→身份。联盟颜色决定red/blue的敌我(设计稿§0:蓝方视角红球是敌方)。
     * 优先认类别名;类别名读不到(空串)时退回类别号。
     * 类别名非空、却不认识 → 返回null(这帧不追):宁可漏一个球,也不能认错敌我*/
    private BallTarget.Kind mapKind(String name, int classId) {
        if (name == null || name.trim().isEmpty()) {
            switch (classId) {
                case ID_RED:    name = "red";    break;
                case ID_YELLOW: name = "yellow"; break;
                case ID_BLUE:   name = "blue";   break;
                default:        return null;
            }
        }
        switch (name.trim().toLowerCase(Locale.ROOT)) {
            case "yellow": return BallTarget.Kind.POLLEN;
            case "blue":   return allianceRed ? BallTarget.Kind.ENEMY_BIG : BallTarget.Kind.ALLY_BIG;
            case "red":    return allianceRed ? BallTarget.Kind.ALLY_BIG  : BallTarget.Kind.ENEMY_BIG;
            default:       return null;
        }
    }
}