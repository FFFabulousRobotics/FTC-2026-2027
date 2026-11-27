package org.firstinspires.ftc.teamcode.Auto;

/**选球器 + 接近控制律。纯逻辑:不碰硬件,时间戳和机器人朝向都由调用方传入,所以能在电脑上离线跑(Test/OfflineTest)。
 * 分工:BallTracker 管"球在哪、是谁、可不可信",这个类管"先追哪一颗"和"怎么开过去"*/
public class IntakeSelector {

    /*-------- 参数墙:评分(设计稿§1,调参只动这里) --------*/
    public static final int    SCORE_ALLY      = 100;  //我方大球基础分
    public static final int    SCORE_POLLEN    = 60;   //花粉基础分
    public static final double D_MAX_CM        = 250;  //视觉有效距离
    public static final double D_FLOOR         = 0.15; //距离保底:远球降分不清零
    public static final double CONF_POW        = 2;    //f_c = c²。置信度门 CONF_MIN 在 BallTracker 那边管(能不能追)
    public static final double ANGLE_DECAY     = 0.5;  //画面边缘 ≈0.5
    public static final double FOV_DEG         = 95;   //水平视场角,换镜头必须改
    public static final double SWITCH_HYST     = 1.25; //新目标领先 25% 才允许换(防横跳)
    public static final double MEMORY_DISCOUNT = 0.5;  //靠记忆(这一帧没亲眼看到)的球打五折

    /*-------- 参数墙:挡路检测 --------*/
    public static final double CORRIDOR_HALF_CM = 15;  //吸入口半宽 + 球半径:这么窄的走廊里有敌球就算挡路
    public static final double BLOCK_MARGIN_CM  = 10;  //候选球身后这么近的敌球也算挡(球有直径)

    /*-------- 参数墙:接近控制 --------*/
    public static final double EDGE_DEG       = 38;   //偏离车头超过这个角 = 画面边缘,要先转过去看清(VERIFY_SPIN)
    public static final double MAX_DRIVE      = 0.5;  //离得远时的前进功率
    public static final double SLOW_WITHIN_CM = 120;  //进入这个距离后开始线性减速
    public static final double MIN_SPEED      = 0.3;  //减速下限(占MAX_DRIVE的比例),免得爬着永远到不了
    public static final double K_TURN         = 0.016;//转向比例:偏1度给多少功率(K_TURN × EDGE_DEG ≈ MAX_TURN)
    public static final double MAX_TURN       = 0.6;
    public static final double K_STRAFE       = 0.02; //横移辅助比例(麦轮不用等车头转到位就能往侧面靠)
    public static final double MAX_STRAFE     = 0.4;

    /*-------- 参数墙:相机相对"吸入口中心"的安装偏移(单位厘米;TODO: 上车用尺子量) --------
     * 相机一般不会正好装在吸入口的正中心。算法要的是"球相对吸入口中心在哪",
     * 相机给的却是"球相对相机在哪",所以要把这段偏移补上。
     * 轴向和 pedroPathing/Constants.java 里 Pinpoint 偏移的规定一致:
     *   x:前后。相机在吸入口中心的前方为正,后方为负
     *   y:左右。相机在吸入口中心的左边为正,右边为负
     *   z:高低。相机镜头在吸入口中心的上方为正*/
    public static double CAM_OFFSET_X_CM = 0;//TODO: 实测后填。0 = 相机和吸入口中心前后对齐
    public static double CAM_OFFSET_Y_CM = 0;//TODO: 实测后填。0 = 相机和吸入口中心左右对齐
    public static double CAM_OFFSET_Z_CM = 0;//TODO: 实测后填。目前没有代码读它,只是预留:
                                              //距离是沿着地面标定的,平面上的几何用不到高度。
                                              //以后要按"视线距离"换算地面距离、或者算俯仰角的时候,才用得上

    /*================= 选球 =================*/

    /**误差角:车头还要往右转多少度才对准这颗球(右正)。符号约定见 BallTracker*/
    public static double headingErrDeg(double yawDeg, BallTracker.Track t) {
        return BallTracker.headingErrDeg(yawDeg, t.bearingDeg);
    }

    /**评分 W = S × f_d × f_c × f_a × f_m(设计稿§1)。
     * 乘法模型:身份项(S、f_c)能一票否决;后勤项(f_d、f_a)只能软性降分;
     * f_m 是新加的:这一帧没亲眼看到、只凭记忆的球打五折——优先追眼前确实看得见的*/
    public static double score(BallTracker.Track t, double yawDeg, double pollenBoost) {
        double s  = (t.kind == BallTarget.Kind.ALLY_BIG) ? SCORE_ALLY : SCORE_POLLEN * pollenBoost;
        double fd = clamp(1.0 - t.distCm / D_MAX_CM, D_FLOOR, 1.0);
        double fc = Math.pow(t.conf, CONF_POW);
        double fa = 1.0 - ANGLE_DECAY * Math.min(Math.abs(headingErrDeg(yawDeg, t)) / (FOV_DEG / 2.0), 1.0);
        double fm = t.seenThisFrame ? 1.0 : MEMORY_DISCOUNT;
        return s * fd * fc * fa * fm;
    }

    /**身份门:这颗球"敢不敢吸"。只看身份,不看它现在在不在眼前(盲吸那一步也要过这道门)*/
    public static boolean identityOk(BallTracker tracker, BallTracker.Track t, long nowMs) {
        if (t.everEnemy) return false;                       //敌球,或者曾经有一帧像敌球
        if (!BallTracker.confSteady(t)) return false;          //没持续看清 = 当敌方处理(宁漏勿错)
        return !tracker.isBanned(t.bearingDeg, nowMs);       //这个方位吐过球
    }

    /**候选门:五道全过才算候选。@param allowEdge 画面边缘的球算不算(只有允许 VERIFY_SPIN 时才算)*/
    public static boolean isCandidate(BallTracker tracker, BallTracker.Track t, double yawDeg, long nowMs, boolean allowEdge) {
        if (!identityOk(tracker, t, nowMs)) return false;
        if (!BallTracker.isConfirmed(t, nowMs)) return false;   //还没确认:可能只是闪烁
        if (!BallTracker.isLive(t, nowMs)) return false;        //已经丢了
        if (!allowEdge && Math.abs(headingErrDeg(yawDeg, t)) > EDGE_DEG) return false;
        return !pathBlocked(tracker, t, nowMs);                 //去它的路上有敌球
    }

    /**挡路检测:把敌球的位置投到"我 → 候选球"这条线上。
     * 敌球比候选球近、且离这条线不到 CORRIDOR_HALF_CM → 挡路,这个候选不选。
     * 按几何算,不按"角度差不多"猜:正前方的敌球挡路,旁边30cm的不挡;在候选球身后的也不挡。
     * 被"记仇"过的球(everEnemy)一律当敌球对待*/
    public static boolean pathBlocked(BallTracker tracker, BallTracker.Track t, long nowMs) {
        for (BallTracker.Track e : tracker.tracks()) {
            if (e == t || !e.everEnemy) continue;
            if (!BallTracker.isConfirmed(e, nowMs) || !BallTracker.isLive(e, nowMs)) continue;
            double phi = Math.toRadians(BallTracker.wrap180(t.bearingDeg - e.bearingDeg));
            double along = e.distCm * Math.cos(phi);            //敌球在这条线上走了多远
            double lateral = Math.abs(e.distCm * Math.sin(phi)); //敌球离这条线多远
            if (along > 0 && along < t.distCm + BLOCK_MARGIN_CM && lateral < CORRIDOR_HALF_CM) return true;
        }
        return false;
    }

    /**主入口:本帧该追谁。没有可选的返回 null。
     * @param heldId     正在追的那颗球的航迹编号,没有传 -1
     * @param allowEdge  是否允许选出画面边缘的球(为 true 时调用方应先 VERIFY_SPIN 再接近)*/
    public static BallTracker.Track select(BallTracker tracker, double yawDeg, double pollenBoost, long nowMs,
                                           int heldId, boolean allowEdge) {
        BallTracker.Track best = null;
        double bestW = 0;
        for (BallTracker.Track t : tracker.tracks()) {
            if (!isCandidate(tracker, t, yawDeg, nowMs, allowEdge)) continue;
            double w = score(t, yawDeg, pollenBoost);
            if (w > bestW) { bestW = w; best = t; }
        }
        /*滞回(设计稿§2):正在追的那颗还合格时,新目标必须领先 25% 才换,防止两颗球分数接近时来回横跳。
         * 正在追的那颗已经出局(丢了/被记仇/被挡路)时不走这条,立刻改追最好的*/
        BallTracker.Track held = tracker.find(heldId);
        if (best != null && held != null && held != best
                && isCandidate(tracker, held, yawDeg, nowMs, allowEdge)
                && bestW < score(held, yawDeg, pollenBoost) * SWITCH_HYST) {
            return held;
        }
        return best;
    }

    /*================= 相机偏移:把"相机看到的"换算成"相对吸入口中心的" =================*/

    /**球相对吸入口中心在哪,返回 {前方多少厘米, 右边多少厘米}。
     * 相机告诉我们的只有"球离相机多远、偏右多少度";加上相机自己的位置,才是球相对吸入口中心的位置。
     * @param camAheadCm 相机在吸入口中心前方多少厘米(就是 CAM_OFFSET_X_CM)
     * @param camLeftCm  相机在吸入口中心左边多少厘米(就是 CAM_OFFSET_Y_CM)。
     *                   注意符号:这两个是"前、左为正",而算法里的偏角和横移都是"右为正",所以下面 y 取了负号*/
    public static double[] ballFromIntake(double camErrDeg, double camDistCm, double camAheadCm, double camLeftCm) {
        double rad = Math.toRadians(camErrDeg);
        return new double[] {
                camAheadCm + camDistCm * Math.cos(rad),
                -camLeftCm + camDistCm * Math.sin(rad)
        };
    }

    /**从吸入口中心看,球偏右多少度。也就是:车头要往右转多少度,正对它的才是吸入口,而不是相机的正前方。
     * 相机和吸入口中心对齐时,它就等于相机偏角本身(原样返回,一个数都不差)*/
    public static double aimErrDeg(double camErrDeg, double camDistCm, double camAheadCm, double camLeftCm) {
        if (camAheadCm == 0 && camLeftCm == 0) return camErrDeg;
        double[] p = ballFromIntake(camErrDeg, camDistCm, camAheadCm, camLeftCm);
        if (p[0] <= 0) return 0;//球已经到了吸入口中心这个深度:没有"往哪边转"可说了,别让它算出一个180度原地打转
        return Math.toDegrees(Math.atan2(p[1], p[0]));
    }

    /**球离吸入口中心多远(厘米)。"够不够近可以开吸"、"要拱多久",看的都是它,不是相机测到的距离*/
    public static double reachCm(double camErrDeg, double camDistCm, double camAheadCm, double camLeftCm) {
        if (camAheadCm == 0 && camLeftCm == 0) return camDistCm;
        double[] p = ballFromIntake(camErrDeg, camDistCm, camAheadCm, camLeftCm);
        return Math.hypot(p[0], p[1]);
    }

    /**上面两个函数的常用写法:偏移直接取参数墙里的值*/
    public static double aimErrDeg(double camErrDeg, double camDistCm) {
        return aimErrDeg(camErrDeg, camDistCm, CAM_OFFSET_X_CM, CAM_OFFSET_Y_CM);
    }
    public static double reachCm(double camErrDeg, double camDistCm) {
        return reachCm(camErrDeg, camDistCm, CAM_OFFSET_X_CM, CAM_OFFSET_Y_CM);
    }

    /*================= 接近控制 =================*/

    /**接近控制律:输入"车头还要往右转多少度"和距离,输出 {前进, 右移, 右转} 三个功率分量。
     * 符号约定和 ManualOpMode 完全一样(推摇杆右 = 右转/右移),所以球在右边 → 三个分量全为正。
     * 三条规则:
     *  - 对准度 align = 1 − |误差角|/EDGE_DEG:偏得越远越以转向为主,偏出 EDGE_DEG 就原地转、不往前冲
     *  - 速度随距离线性递减(SLOW_WITHIN_CM 内),但有下限,免得爬着永远到不了
     *  - 横移只做辅助:比例于误差角 × 对准度,没对准时不乱平移*/
    public static double[] approachDrive(double errDeg, double distCm) {
        double align = clamp(1.0 - Math.abs(errDeg) / EDGE_DEG, 0.0, 1.0);
        double speed = MAX_DRIVE * clamp(distCm / SLOW_WITHIN_CM, MIN_SPEED, 1.0);
        return new double[] {
                speed * align,
                clamp(K_STRAFE * errDeg, -MAX_STRAFE, MAX_STRAFE) * align,
                clamp(K_TURN * errDeg, -MAX_TURN, MAX_TURN)
        };
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
