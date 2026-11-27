package org.firstinspires.ftc.teamcode.Auto;

/**相机一帧画面里的一个检测框。视觉层(Limelight.pollFrame)每帧生成一批,决策层只读不改。
 * id 只是这一帧里的序号;跨帧认球是 BallTracker 的事(那边用 Track.id)*/
public class BallTarget {
    /**球的身份:颜色分类的产物。ENEMY_BIG 是负目标,永不进入候选(设计稿§0)*/
    public enum Kind { ALLY_BIG, POLLEN, ENEMY_BIG }

    public final Kind   kind;        //球种
    public final int    id;          //帧内序号
    public final double txDeg;       //相机系偏角(度):球在画面中心右侧为正
    public final double bearingDeg;  //场地系方位(度) = 机器人yaw − tx。机器人转动不会改变静止球的这个值
    public final double ta;          //检测框占画面的面积百分比(0-100)
    public final double distCm;      //由ta反推的估计距离 d = K/√ta(标定见 Test/BallKCalib)
    public final double conf;        //分类置信度 0-1

    public BallTarget(Kind kind, int id, double txDeg, double bearingDeg, double ta, double distCm, double conf) {
        this.kind = kind; this.id = id; this.txDeg = txDeg; this.bearingDeg = bearingDeg;
        this.ta = ta; this.distCm = distCm; this.conf = conf;
    }

    @Override public String toString() {
        return kind + "#" + id + String.format(" tx=%.1f° d=%.0fcm c=%.2f", txDeg, distCm, conf);
    }
}
