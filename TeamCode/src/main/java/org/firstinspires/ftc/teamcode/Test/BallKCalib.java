package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto.BallTarget;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*标定距离常数K,顺便当"球检测诊断页"用——AutoIntake 上车之前先跑这个。
 *
 第一件事不是标定,是核对标签:拿红/黄/蓝三种球分别举到镜头前,看telemetry的"原始检测"一行:
   [类别名|id类别号|c置信度|tx偏角|ta面积]
   - 类别名应该是 red / yellow / blue。如果是空的,说明SDK读不到名字,程序会改按类别号认:
     id0=红 id1=黄 id2=蓝(训练集 classes.txt 的顺序)——这时务必确认号码和颜色对得上
   - 置信度应该是 0-100 的数(比如 93)
   - 球举到车头右边,tx 应该是正的
 任何一条不对,先解决它,再去标定。
 *
 然后标定:改KNOWN_CM → 把球摆在那么远 → 按A采样 → 换距离(50/100/150cm)重复 → 把均值抄进 Limelight.K_POLLEN / K_BIG
 黄色球和大球是两个K,分别标。标完复核:摆已知距离,看"检测"一行里的d估得准不准*/
@TeleOp(name = "Ball K Calib", group = "Test")
public class BallKCalib extends OpMode {
    private static final double KNOWN_CM = 100;//TODO: 每轮采样前改成实际摆放距离

    private Limelight vision;
    private List<BallTarget> latest = new ArrayList<>();//最近一帧的副本
    private final List<Double> kPollen = new ArrayList<>();
    private final List<Double> kBig = new ArrayList<>();
    private boolean wasA = false;

    @Override public void init() {
        vision = new Limelight(this);
        boolean ok = vision.usePipeline(Limelight.PIPE_BALLS);
        vision.start();
        telemetry.addData("切管线", ok ? "已切到 " + Limelight.PIPE_BALLS + " 号" : "失败!查相机供电/网线");
        telemetry.update();
    }

    @Override public void loop() {
        if (vision.pollFrame(vision.getYawDeg())) latest = new ArrayList<>(vision.getBallTargets());//出了新帧才刷新(每帧只读一次)

        boolean nowA = gamepad1.a;
        if (nowA && !wasA) {//只在刚按下的那一帧采样,按住不放不会连采
            for (BallTarget t : latest) {
                double k = KNOWN_CM * Math.sqrt(t.ta);//由 d = K/√ta 反解 K
                if (t.kind == BallTarget.Kind.POLLEN) kPollen.add(k); else kBig.add(k);
            }
        }
        wasA = nowA;

        telemetry.addData("距离", "%.0fcm(改代码里的KNOWN_CM)", KNOWN_CM);
        telemetry.addData("原始检测", vision.getRawText().isEmpty() ? "(没有检测)" : vision.getRawText());
        for (BallTarget t : latest)
            telemetry.addData("检测", String.format(Locale.ROOT, "%s tx=%.1f° 场地%.0f° ta=%.2f d=%.0fcm c=%.2f",
                    t.kind, t.txDeg, t.bearingDeg, t.ta, t.distCm, t.conf));
        telemetry.addData("花粉样本", "%d 个, K均值 = %.1f", kPollen.size(), mean(kPollen));
        telemetry.addData("大球样本", "%d 个, K均值 = %.1f", kBig.size(), mean(kBig));
        telemetry.addData("yaw", "%.1f(车头左转应变大)", vision.getYawDeg());
        telemetry.addData("相机", vision.isConnected()
                ? String.format(Locale.ROOT, "连通 %.0f帧/秒", vision.getFps()) : "断开!");
        telemetry.update();
    }

    @Override public void stop() { vision.stop(); }

    private double mean(List<Double> list) {
        if (list.isEmpty()) return 0;
        double s = 0; for (double v : list) s += v;
        return s / list.size();
    }
}
