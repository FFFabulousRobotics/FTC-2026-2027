package org.firstinspires.ftc.teamcode.Auto;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**球跟踪器:把"每一帧各自独立的检测框"串成"场上同一颗球的连续历史"。纯逻辑:不碰硬件,时间戳由调用方传入。
 *
 * v1.0 每帧独立选球,只看眼前这一帧。这个类补上时间维度,做四件事:
 *  1. 确认:连续被看到够久、够多帧,才算可信目标——单帧闪烁骗不了它
 *  2. 容忍:漏看一小会儿不算丢,还是原来那颗,不用重新确认——机器人不会一顿一顿
 *  3. 记仇:一颗球只要有一帧被看成敌球,到被忘记为止都不再追(宁漏勿错)
 *  4. 拉黑:吸错吐球后按场地系方位记下"那个方向别去",机器人转了身也依然有效
 *
 * 跨帧认球用"场地系方位 + 距离":机器人原地转圈时,静止球的场地方位不变,所以转动不会把球认丢*/
public class BallTracker {

    /*-------- 参数墙:时间与关联(调参只动这里) --------*/
    public static final long   CONFIRM_MS         = 120;  //连续被看到这么久才算"确认"
    public static final int    CONFIRM_MIN_FRAMES = 2;    //且至少看到这么多帧
    public static final long   LOST_MS            = 300;  //多久没看到算"丢失"(丢失前靠记忆继续追)
    public static final long   MEMORY_MS          = 3000; //多久没看到就彻底忘掉
    public static final double ASSOC_GATE_DEG     = 12;   //方位差在此之内才可能是同一颗球
    public static final double ASSOC_GATE_CM      = 40;   //距离差在此之内才可能是同一颗球
    public static final double CONF_SMOOTH        = 0.5;  //置信度平滑:新 = 旧 + 0.5×(这次看到的 − 旧值)。只用来打分
    public static final double CONF_MIN           = 0.6;  //置信度门(设计稿§3):单帧低于此值 = 这一帧没看清
    public static final int    CONF_OK_WINDOW     = 8;    //置信度也要讲持续性:看最近这么多次
    public static final int    CONF_OK_MIN        = 7;    //其中至少这么多次是看清的,才算"看清了"(能不能追看这个,不看平滑值)
    public static final double BLACK_BEAM_DEG     = 20;   //拉黑的扇形半宽
    public static final long   BAN_FIRST_MS       = 2000; //第一次:拉黑这么久
    public static final long   BAN_REPEAT_MS      = 60000;//同一方位第二次:本场别再来了
    public static final long   STRIKE_MEMORY_MS   = 20000;//拉黑过期后还记这么久,用来认出"同一方位又吐了"

    /**一颗球(的一条航迹):跨帧不变的只有 id,其余都是最近一次看到时的状态*/
    public static class Track {
        public final int id;          //跨帧不变的编号
        public BallTarget.Kind kind;  //最近一次看到的身份
        public boolean everEnemy;     //只要有一帧被判成敌球就置位,之后不再追
        public double bearingDeg;     //场地系方位(度)
        public double distCm;         //估计距离
        public double conf;           //平滑后的置信度(打分用)
        public int confOkBits;        //最近8次看到,每次置信度过没过门(1=过)。每看到一次左移一格。能不能追看这个
        public int hits;              //本段连续被看到的帧数
        public long streakStartMs;    //本段连续被看到是从何时开始的
        public long lastSeenMs;       //最近一次被看到的时刻
        public boolean seenThisFrame; //最近处理的这一帧里是否亲眼看到

        Track(int id) { this.id = id; }
    }

    /**一处拉黑:bearingDeg 这个方位在 untilMs 之前不追*/
    public static class Ban {
        public final double bearingDeg;
        public long untilMs;
        public int strikes;          //同一方位已经吐过几次球
        Ban(double bearingDeg, long untilMs) { this.bearingDeg = bearingDeg; this.untilMs = untilMs; }
    }

    private final List<Track> tracks = new ArrayList<>();
    private final List<Ban> bans = new ArrayList<>();
    private int nextId = 1;

    /*-------- 角度换算:符号约定全程序只在这里定义一次 --------
     * 朝向和方位:逆时针为正(与 IMU 一致)。相机偏角 tx:球在画面右侧为正。
     * 方位 = yaw − tx,所以 误差角 = yaw − 方位 = tx = "车头还要往右转多少度"*/

    /**把角度折回 -180~180(350° 和 -10° 是同一个方向)*/
    public static double wrap180(double deg) {
        double a = deg % 360.0;
        if (a >= 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }

    /**误差角:车头还要往右转多少度才对准这个方位(右正,范围 -180~180)*/
    public static double headingErrDeg(double yawDeg, double bearingDeg) {
        return wrap180(yawDeg - bearingDeg);
    }

    /**球在场地上的方位:方位 = 机器人朝向 − 相机偏角(球在画面右边,它的方位就比车头小一点)。
     * Limelight封装和OfflineTest都只调用这一处,公式不抄第二遍*/
    public static double bearingOf(double yawDeg, double txDeg) {
        return wrap180(yawDeg - txDeg);
    }

    /**确认过 = 本段连续被看到 ≥ CONFIRM_MS,且 ≥ CONFIRM_MIN_FRAMES 帧*/
    public static boolean isConfirmed(Track t, long nowMs) {
        return t.hits >= CONFIRM_MIN_FRAMES && nowMs - t.streakStartMs >= CONFIRM_MS;
    }

    /**没丢 = 最近 LOST_MS 之内还看到过。丢了之前靠记忆继续追,不中途变卦*/
    public static boolean isLive(Track t, long nowMs) {
        return nowMs - t.lastSeenMs <= LOST_MS;
    }

    /**看清了 = 最近几次看到里,几乎每次置信度都过门。
     * 刚出现的航迹要求帧帧看清;看满 CONF_OK_WINDOW 帧之后,才允许偶尔有一帧没看清。
     * 为什么要这么绕:只看单帧,真清楚的球偶尔糊一帧就被踢出候选(机器人一顿一顿);
     * 只看平滑值,真模糊的球偶尔清楚一帧就混进来了(可能因此吸错)。看"最近8帧里有7帧清楚",两头都躲开*/
    public static boolean confSteady(Track t) {
        int seen = Math.min(t.hits, CONF_OK_WINDOW);
        return Integer.bitCount(t.confOkBits) >= Math.min(seen, CONF_OK_MIN);
    }

    /**当前所有航迹。只许读,不许改(要改也通过下面的方法)*/
    public List<Track> tracks() { return tracks; }
    public List<Ban> bans()     { return bans; }

    public Track find(int id) {
        for (Track t : tracks) if (t.id == id) return t;
        return null;
    }

    /**全部清空(开场时调一次)*/
    public void reset() {
        tracks.clear();
        bans.clear();
    }

    /**喂一帧新的相机画面。只在相机出了新帧时调用:同一帧喂两遍帧数会虚高,确认机制就白设了。
     * @param frame  这一帧的检测框,没检测到就传空表*/
    public void observe(List<BallTarget> frame, long nowMs) {
        for (Track t : tracks) t.seenThisFrame = false;
        forgetOld(nowMs);

        int n = frame.size(), m = tracks.size();
        double[][] cost = new double[n][m];
        boolean[] detUsed = new boolean[n], trkUsed = new boolean[m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                cost[i][j] = pairCost(frame.get(i), tracks.get(j));

        /*配对:每次取代价最小的一对。代价 ≤ 1 才在关联门内,否则是 NaN,比不出来*/
        for (int round = 0; round < Math.min(n, m); round++) {
            int bi = -1, bj = -1;
            double best = 1.0;
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++)
                    if (!detUsed[i] && !trkUsed[j] && cost[i][j] <= best) { best = cost[i][j]; bi = i; bj = j; }
            if (bi < 0) break;
            detUsed[bi] = true;
            trkUsed[bj] = true;
            absorb(tracks.get(bj), frame.get(bi), nowMs);
        }

        /*没配上任何航迹的检测框 = 新出现的球*/
        for (int i = 0; i < n; i++) {
            if (detUsed[i]) continue;
            Track t = new Track(nextId++);
            absorb(t, frame.get(i), nowMs);
            tracks.add(t);
        }
    }

    /**两个东西是同一颗球的代价:方位差、距离差各按自己的门归一化,合起来 ≤ 1 才算门内*/
    private static double pairCost(BallTarget d, Track t) {
        double dAng = Math.abs(wrap180(d.bearingDeg - t.bearingDeg)) / ASSOC_GATE_DEG;
        double dDist = Math.abs(d.distCm - t.distCm) / ASSOC_GATE_CM;
        double c = Math.hypot(dAng, dDist);
        return c <= 1.0 ? c : Double.NaN;
    }

    /**把一次看到并入航迹*/
    private static void absorb(Track t, BallTarget d, long nowMs) {
        if (t.hits == 0 || nowMs - t.lastSeenMs > LOST_MS) {   //新出现的球,或中断了一小会儿才又看到:重新数
            t.hits = 0;
            t.streakStartMs = nowMs;
            t.confOkBits = 0;
        }
        t.hits++;
        t.confOkBits = ((t.confOkBits << 1) | (d.conf >= CONF_MIN ? 1 : 0)) & 0xFF;
        t.bearingDeg = d.bearingDeg;
        t.distCm = d.distCm;
        t.conf = (t.hits == 1) ? d.conf : t.conf + CONF_SMOOTH * (d.conf - t.conf);
        t.kind = d.kind;
        if (d.kind == BallTarget.Kind.ENEMY_BIG) t.everEnemy = true;
        t.lastSeenMs = nowMs;
        t.seenThisFrame = true;
    }

    /**把一颗已知的球标成"敌球":视觉说它像友军、但吸进口发现是敌球时用(吸错吐球之后)*/
    public void markEnemy(int id) {
        Track t = find(id);
        if (t != null) t.everEnemy = true;
    }

    /**拉黑一个场地系方位。同一方位第二次拉黑,时长从 BAN_FIRST_MS 跳到 BAN_REPEAT_MS*/
    public void ban(double bearingDeg, long nowMs) {
        pruneBans(nowMs);
        for (Ban b : bans) {
            if (Math.abs(wrap180(bearingDeg - b.bearingDeg)) < BLACK_BEAM_DEG) {
                b.strikes++;
                b.untilMs = nowMs + (b.strikes >= 2 ? BAN_REPEAT_MS : BAN_FIRST_MS);
                return;
            }
        }
        Ban fresh = new Ban(bearingDeg, nowMs + BAN_FIRST_MS);
        fresh.strikes = 1;//这一次就算第一次,下一次再来就是第二次
        bans.add(fresh);
    }

    public boolean isBanned(double bearingDeg, long nowMs) {
        for (Ban b : bans)
            if (nowMs < b.untilMs && Math.abs(wrap180(bearingDeg - b.bearingDeg)) < BLACK_BEAM_DEG) return true;
        return false;
    }

    /**过期的拉黑不马上删:还要留 STRIKE_MEMORY_MS,才认得出"同一个方位又吐了一次"。只在这里删,列表不会无限长*/
    private void pruneBans(long nowMs) {
        Iterator<Ban> it = bans.iterator();
        while (it.hasNext())
            if (it.next().untilMs + STRIKE_MEMORY_MS <= nowMs) it.remove();
    }

    private void forgetOld(long nowMs) {
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext())
            if (nowMs - it.next().lastSeenMs > MEMORY_MS) it.remove();
    }
}
