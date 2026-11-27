package org.firstinspires.ftc.teamcode.Test;

import org.firstinspires.ftc.teamcode.Auto.BallTarget;
import org.firstinspires.ftc.teamcode.Auto.BallTracker;
import org.firstinspires.ftc.teamcode.Auto.IntakeFsm;
import org.firstinspires.ftc.teamcode.Auto.IntakeSelector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;

/**离线测试:不需要机器人,在电脑上直接跑(Android Studio 里右键 Run 'OfflineTest.main()')。
 * 每个场景对应算法的一个机制,自动判定通过/失败,退出码非0 = 有失败。
 * 改了 IntakeFsm / IntakeSelector / BallTracker 的参数墙之后,先跑这个,再上车。
 *
 * 前半部分测跟踪器和选球器(纯逻辑),后半部分用"假机器人"整段整段地跑状态机。
 * 覆盖不到的只剩 Limelight 封装本身——那部分要硬件,用 Test/BallKCalib 上车核对*/
public class OfflineTest {
    private static int passed = 0, failed = 0;

    public static void main(String[] args) {
        /*下面这些场景(除了 fsmCameraOffset)都假设"相机和吸入口中心对齐"。
          真实机器人上量出来的偏移各不相同,所以开头先归零:量完偏移、填进参数墙之后,这个测试照样应该是全绿。
          各种非 0 的偏移由 fsmCameraOffset 自己设、自己测、自己还原*/
        IntakeSelector.CAM_OFFSET_X_CM = 0;
        IntakeSelector.CAM_OFFSET_Y_CM = 0;

        scoringOrder();
        confirmationNeedsPersistence();
        dropoutTolerance();
        enemyMemoryVeto();
        pathBlockedByEnemy();
        hysteresis();
        blacklistSurvivesRotation();
        banEscalation();
        angleMath();
        approachControlLaw();
        cameraOffsetGeometry();
        blindIntakeGuard();
        confidencePersistence();

        /*-------- 状态机(用假机器人整段地跑) --------*/
        fsmNormalPickup();
        fsmEjectsEnemy();
        fsmIntakeTimeout();
        fsmIntakeGates();
        fsmCameraOffset();
        fsmArrivalEdge();
        fsmEdgeTarget();
        fsmBlindIntake();
        fsmFailSafes();
        fsmRandomStress();

        System.out.printf("%n通过 %d 项,失败 %d 项%n", passed, failed);
        if (failed > 0) System.exit(1);
    }

    /*================= 场景 =================*/

    /**设计稿§1.3:近的花粉优先于远的我方大球;敌球再近也不进候选*/
    private static void scoringOrder() {
        header("评分排序:近花粉 > 远大球;敌球再近也不进候选");
        Sim s = new Sim();
        BallTracker.Track pick = null;
        for (int i = 0; i < 5; i++)
            pick = s.step(50, s.ball(BallTarget.Kind.POLLEN, 3, 50, 0.90),
                               s.ball(BallTarget.Kind.ALLY_BIG, -10, 200, 0.85),
                               s.ball(BallTarget.Kind.ENEMY_BIG, -40, 40, 0.95));
        for (BallTracker.Track t : s.tracker.tracks())
            System.out.printf("  %-9s 距离%4.0fcm  W=%.1f%n", t.kind, t.distCm, IntakeSelector.score(t, s.yaw, 1.0));
        check("选中近花粉", pick != null && pick.kind == BallTarget.Kind.POLLEN);
        double wPollen = IntakeSelector.score(pick, s.yaw, 1.0);
        Sim near = new Sim();
        BallTracker.Track bigNear = null;
        for (int i = 0; i < 5; i++) bigNear = near.step(50, near.ball(BallTarget.Kind.ALLY_BIG, 0, 60, 0.85));
        check("逻辑不用改:同一颗大球挪到60cm就反超",
                bigNear != null && IntakeSelector.score(bigNear, near.yaw, 1.0) > wPollen);
        check("末段花粉加权(×1.5)进一步拉开", IntakeSelector.score(pick, s.yaw, 1.5) > 1.4 * wPollen);

        Sim faraway = new Sim();
        BallTracker.Track far = null;
        for (int i = 0; i < 5; i++) far = faraway.step(50, faraway.ball(BallTarget.Kind.ALLY_BIG, 0, 300, 0.90));//比 D_MAX_CM 还远
        check("距离因子有保底(D_FLOOR):比 D_MAX_CM 还远的球,分数不会是 0,照样能被选中",
                far != null && far.distCm > IntakeSelector.D_MAX_CM && IntakeSelector.score(far, faraway.yaw, 1.0) > 0);

        Sim e = new Sim();
        BallTracker.Track enemyOnly = null;
        for (int i = 0; i < 10; i++) enemyOnly = e.step(50, e.ball(BallTarget.Kind.ENEMY_BIG, 0, 40, 0.99));
        check("只有一颗又近又清楚的敌球:看再久也不选", enemyOnly == null);
    }

    /**一闪而过的检测不可能是可信目标:要被看到够久、够多帧才算确认*/
    private static void confirmationNeedsPersistence() {
        header("确认:一闪而过的检测骗不过");
        Sim s = new Sim();
        check("只看到1帧:不选", s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95)) == null);
        check("2帧、累计50ms:不选(要满120ms)", s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95)) == null);
        check("3帧、累计150ms:可选", s.step(100, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95)) != null);

        Sim f = new Sim();
        BallTracker.Track pick = null;
        for (int i = 0; i < 20; i++) {//同一个位置反复出现,但每次都只闪一帧、然后消失350ms
            pick = f.step(50, f.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));
            f.step(350);
        }
        check("同一处闪了20次、每次只有1帧:始终不选", pick == null);

        Sim once = new Sim();
        once.step(50, once.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));//只看到过这一帧
        once.now += 150;//之后一直没再看到它;但还在记忆期内,所以还没算丢
        check("只看到过1帧:过了150毫秒(时间够了)也确认不了,因为帧数不够",
                !BallTracker.isConfirmed(once.tracker.tracks().get(0), once.now)
                        && BallTracker.isLive(once.tracker.tracks().get(0), once.now));

        Sim d = new Sim();
        BallTracker.Track steady = null;
        for (int i = 0; i < 7; i++)//反过来:3帧里漏1帧,但一直在那个位置
            steady = (i % 3 == 2) ? d.step(50) : d.step(50, d.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));
        check("3帧漏1帧、持续了350ms的球:算数(偶尔漏看是要容忍的)", steady != null);
    }

    /**漏看一小会儿不算丢:还是同一颗球、不用重新确认,机器人就不会一顿一顿*/
    private static void dropoutTolerance() {
        header("丢失容忍:漏看200ms照追,漏看350ms才算丢");
        Sim s = new Sim();
        BallTracker.Track t = null;
        for (int i = 0; i < 5; i++) t = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));
        int id = t.id;
        double wSeen = IntakeSelector.score(t, s.yaw, 1.0);

        BallTracker.Track during = s.step(200);
        check("漏看200ms:还在追,而且是同一颗", during != null && during.id == id && !during.seenThisFrame);
        check("凭记忆的球分数打五折", during != null
                && Math.abs(IntakeSelector.score(during, s.yaw, 1.0) - 0.5 * wSeen) < 1e-9);
        check("累计350ms:算丢了", s.step(150) == null);
        check("400ms后又看到:要重新确认", s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95)) == null);
        check("但编号没变(还记得是它)", s.tracker.tracks().size() == 1 && s.tracker.find(id) != null);

        BallTracker.Track again = null;
        for (int i = 0; i < 3; i++) again = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));
        check("重新确认后可选,还是同一个编号", again != null && again.id == id);

        s.now += BallTracker.MEMORY_MS + 1;
        s.step(1, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));
        check("3秒没见就忘掉:再来算新球", s.tracker.find(id) == null && s.tracker.tracks().size() == 1);
    }

    /**一颗球只要有一帧被看成敌球,就再也不追;别的球要去它身后,也当作被它挡住了*/
    private static void enemyMemoryVeto() {
        header("记仇:有一帧像敌球,就再也不追");
        Sim s = new Sim();
        BallTracker.Track pick = null;
        for (int i = 0; i < 5; i++) pick = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));
        check("先正常确认一颗友军大球", pick != null);

        s.step(50, s.ball(BallTarget.Kind.ENEMY_BIG, 0, 100, 0.95));//就这一帧判成了敌球
        for (int i = 0; i < 20; i++) pick = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95));
        check("之后1秒一直判成友军:依然不选", pick == null);

        BallTracker.Track behind = null;
        for (int i = 0; i < 5; i++)
            behind = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.95), s.ball(BallTarget.Kind.ALLY_BIG, 0, 180, 0.95));
        check("它身后那颗的路被它挡住:也不选", behind == null);
    }

    /**去候选球的路上有敌球 → 不选这个候选。按几何算,不按角度猜*/
    private static void pathBlockedByEnemy() {
        header("挡路检测:去候选球的路上有敌球就不去");
        Sim s = new Sim();
        s.yaw = 100;
        BallTarget ally = s.ball(BallTarget.Kind.ALLY_BIG, 10, 150, 0.9);//场地方位90°、150cm
        BallTracker.Track pick = null;
        for (int i = 0; i < 4; i++) pick = s.step(50, ally);
        check("路上没敌球:可选", pick != null);

        BallTarget inFront = s.ball(BallTarget.Kind.ENEMY_BIG, 12, 80, 0.9);//方位88°、80cm:正好挡在半路
        pick = s.step(50, ally, inFront);
        check("敌球才出现1帧(还没确认):还不算挡", pick != null);
        for (int i = 0; i < 4; i++) pick = s.step(50, ally, inFront);
        check("敌球确认了:不选", pick == null);

        BallTarget beside = s.ball(BallTarget.Kind.ENEMY_BIG, 40, 80, 0.9);//方位60°:离路线约40cm
        for (int i = 0; i < 4; i++) pick = s.step(50, ally, beside);
        check("原来那个才消失200ms:还是不选(它可能只是被挡了一下)", pick == null);
        for (int i = 0; i < 4; i++) pick = s.step(50, ally, beside);
        check("消失400ms后:又能选了", pick != null && pick.kind == BallTarget.Kind.ALLY_BIG);

        BallTarget behind = s.ball(BallTarget.Kind.ENEMY_BIG, 9, 220, 0.9);//方位91°:在友军球身后
        for (int i = 0; i < 4; i++) pick = s.step(50, ally, behind);
        check("敌球在候选球身后:不挡路", pick != null);

        BallTarget justBehind = s.ball(BallTarget.Kind.ENEMY_BIG, 10, 158, 0.9);//方位90°、158cm:刚过候选球 8 厘米,不到 BLOCK_MARGIN_CM
        for (int i = 0; i < 4; i++) pick = s.step(50, ally, justBehind);
        check("敌球刚过候选球 8 厘米(不到 BLOCK_MARGIN_CM):照样算挡路", pick == null);
    }

    /**两颗球分数接近时不来回横跳:新的要领先25%才换*/
    private static void hysteresis() {
        header("滞回:新目标强一点点不换,强很多才换");
        Sim s = new Sim();
        BallTarget a = s.ball(BallTarget.Kind.ALLY_BIG, 0, 120, 0.9);   //A:正前方120cm
        BallTarget b = s.ball(BallTarget.Kind.ALLY_BIG, -20, 62, 0.9);  //B:左前方20°、62cm
        BallTracker.Track held = null;
        for (int i = 0; i < 5; i++) held = s.step(50, a);
        int idA = held.id, idB = -1;

        BallTracker.Track pick = null;
        for (int i = 0; i < 5; i++) pick = s.stepHeld(idA, 50, a, b);
        for (BallTracker.Track t : s.tracker.tracks()) if (t.id != idA) idB = t.id;
        double ratio = IntakeSelector.score(s.tracker.find(idB), s.yaw, 1.0)
                     / IntakeSelector.score(s.tracker.find(idA), s.yaw, 1.0);
        System.out.printf("  B/A 分数比 = %.2f%n", ratio);
        check("B分数更高、但不到1.25倍:继续追A", pick != null && pick.id == idA && ratio > 1.0);

        BallTarget bNear = s.ball(BallTarget.Kind.ALLY_BIG, -20, 40, 0.9);
        for (int i = 0; i < 5; i++) pick = s.stepHeld(idA, 50, a, bNear);
        ratio = IntakeSelector.score(s.tracker.find(idB), s.yaw, 1.0)
              / IntakeSelector.score(s.tracker.find(idA), s.yaw, 1.0);
        System.out.printf("  B/A 分数比 = %.2f%n", ratio);
        check("B近了、分数高出25%以上:改追B", pick != null && pick.id == idB && ratio >= IntakeSelector.SWITCH_HYST);
    }

    /**拉黑按场地系方位记:机器人转了身,同一个方位上的球换了画面位置,依然有效(v1.0 在这里比错了坐标系)。
     * 顺便验证跨帧认球用的也是场地方位:转身没有把这颗球认成新球*/
    private static void blacklistSurvivesRotation() {
        header("拉黑:按场地方位记,转身之后依然有效");
        Sim s = new Sim();
        s.tracker.ban(30, s.now);
        BallTracker.Track pick = null;
        for (int i = 0; i < 5; i++) pick = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, -30, 100, 0.9));
        check("被拉黑的方位上有颗'友军球'(视觉认错了):不选", pick == null);

        s.yaw = 60;
        for (int i = 0; i < 5; i++) pick = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 30, 100, 0.9));
        check("机器人转了60°,同一颗球从画面左边挪到右边:依然不选", pick == null);
        check("跟踪器里始终只有这一颗球(转身没把它认成新球)", s.tracker.tracks().size() == 1);

        for (int i = 0; i < 5; i++)
            pick = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 30, 100, 0.9), s.ball(BallTarget.Kind.ALLY_BIG, -30, 120, 0.9));
        check("旁边方位90°的球不受影响", pick != null && pick.bearingDeg == 90);

        s.now += BallTracker.BAN_FIRST_MS;
        for (int i = 0; i < 5; i++) pick = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 30, 100, 0.9));
        check("拉黑过期:又能选了", pick != null && pick.bearingDeg == 30);
    }

    /**同一方位第二次吐球,拉黑时长从2秒跳到60秒——不然会在同一颗球上反复"吸错-吐出"到比赛结束*/
    private static void banEscalation() {
        header("拉黑升级:同一方位第二次,本场不再来");
        Sim s = new Sim();
        s.tracker.ban(30, s.now);
        check("第一次:拉黑 " + BallTracker.BAN_FIRST_MS + "ms",
                s.tracker.bans().size() == 1 && s.tracker.bans().get(0).untilMs - s.now == BallTracker.BAN_FIRST_MS);
        s.now += BallTracker.BAN_FIRST_MS + 1000;
        s.tracker.ban(35, s.now);
        check("3秒后同一方位(偏5°)又吐一次:拉黑 " + BallTracker.BAN_REPEAT_MS + "ms",
                s.tracker.bans().size() == 1 && s.tracker.bans().get(0).untilMs - s.now == BallTracker.BAN_REPEAT_MS);

        s.tracker.ban(200, s.now);//另一个方位
        s.now += BallTracker.BAN_REPEAT_MS + BallTracker.STRIKE_MEMORY_MS + 1;
        s.tracker.ban(300, s.now);
        check("全都过期很久之后旧记录被清掉:列表不会越积越多", s.tracker.bans().size() == 1);
    }

    /**角度跨±180°时的换算,以及"球在右边 → 误差角为正"的符号约定*/
    private static void angleMath() {
        header("角度换算:跨±180°不出错,球在右边为正");
        check("350° 就是 -10°", BallTracker.wrap180(350) == -10);
        check("-190° 就是 170°", BallTracker.wrap180(-190) == 170);
        check("180° 折成 -180°", BallTracker.wrap180(180) == -180);
        check("车头朝0°、球在-20°(右边20°):误差角 +20", BallTracker.headingErrDeg(0, -20) == 20);
        check("车头朝0°、球在20°(左边20°):误差角 -20", BallTracker.headingErrDeg(0, 20) == -20);
        check("车头朝170°、球在-170°:球在左边20°,误差角 -20", BallTracker.headingErrDeg(170, -170) == -20);
        check("车头朝-170°、球在170°:球在右边20°,误差角 +20", BallTracker.headingErrDeg(-170, 170) == 20);
        check("车头朝30°、球在画面右边10°:球的场地方位是20°", BallTracker.bearingOf(30, 10) == 20);
        check("车头朝10°、球在画面左边20°:球的场地方位是30°", BallTracker.bearingOf(10, -20) == 30);
        check("车头朝170°、球在画面右边20°:方位是150°(不是190)", BallTracker.bearingOf(170, 20) == 150);
        check("车头朝10°、球在正后方(tx=-180):方位写成-170°,不会写成190", BallTracker.bearingOf(10, -180) == -170);
    }

    /**接近控制律的符号(v1.0 在这里把转向写反了)和限幅*/
    private static void approachControlLaw() {
        header("接近控制律:球在右边就右转+右移");
        double[] right = IntakeSelector.approachDrive(20, 200);
        check("球在右前方:右转 + 右移 + 前进", right[2] > 0 && right[1] > 0 && right[0] > 0);
        double[] left = IntakeSelector.approachDrive(-20, 200);
        check("球在左前方:左转 + 左移 + 前进", left[2] < 0 && left[1] < 0 && left[0] > 0);
        double[] edge = IntakeSelector.approachDrive(60, 200);
        check("偏出画面边缘:原地转,不往前冲", edge[0] == 0 && edge[1] == 0 && edge[2] > 0);

        double far = IntakeSelector.approachDrive(0, 300)[0];
        double mid = IntakeSelector.approachDrive(0, 60)[0];
        double near = IntakeSelector.approachDrive(0, 20)[0];
        System.out.printf("  前进功率: 300cm→%.2f   60cm→%.2f   20cm→%.2f%n", far, mid, near);
        check("越近越慢,但不低于下限", far > mid && mid > near
                && near >= IntakeSelector.MAX_DRIVE * IntakeSelector.MIN_SPEED - 1e-9);
        check("三个分量都不超上限", right[0] <= IntakeSelector.MAX_DRIVE + 1e-9
                && Math.abs(right[1]) <= IntakeSelector.MAX_STRAFE + 1e-9
                && Math.abs(right[2]) <= IntakeSelector.MAX_TURN + 1e-9);
        check("误差角179°:转向仍然不超上限",
                Math.abs(IntakeSelector.approachDrive(179, 20)[2]) <= IntakeSelector.MAX_TURN + 1e-9);
    }

    /**盲吸的保险:只有"看不见了"的球允许盲吸;因身份存疑/被拉黑而出局的,绝不允许*/
    private static void blindIntakeGuard() {
        header("盲吸保险:身份存疑的球,再近也不吸");
        Sim s = new Sim();
        BallTracker.Track t = null;
        for (int i = 0; i < 5; i++) t = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 30, 0.95));
        check("已确认的近处友军球:身份没问题", t != null && IntakeSelector.identityOk(s.tracker, t, s.now));
        s.now += 400;
        check("看不见了:算丢失,但身份仍然没问题 → 只有这种情况允许盲吸",
                !BallTracker.isLive(t, s.now) && IntakeSelector.identityOk(s.tracker, t, s.now));
        s.step(50, s.ball(BallTarget.Kind.ENEMY_BIG, 0, 30, 0.95));
        check("但只要有一帧像敌球:禁止盲吸", !IntakeSelector.identityOk(s.tracker, t, s.now));

        Sim low = new Sim();
        BallTracker.Track unsure = null;
        for (int i = 0; i < 5; i++) unsure = low.step(50, low.ball(BallTarget.Kind.ALLY_BIG, 0, 30, 0.50));
        check("置信度只有0.50(低于0.6):选不中,也禁止盲吸",
                unsure == null && !IntakeSelector.identityOk(low.tracker, low.tracker.tracks().get(0), low.now));

        Sim banned = new Sim();
        BallTracker.Track ok = null;
        for (int i = 0; i < 5; i++) ok = banned.step(50, banned.ball(BallTarget.Kind.ALLY_BIG, 0, 30, 0.95));
        banned.tracker.ban(0, banned.now);
        check("方位被拉黑:禁止盲吸", ok != null && !IntakeSelector.identityOk(banned.tracker, ok, banned.now));
    }

    /**置信度也要讲持续性。只看单帧会被骗:真清楚的球偶尔糊一帧,就被踢出候选。
     * 只看平滑值也会被骗:真模糊的球偶尔清楚一帧,平滑值一抬就过门了(绝不能因此去追)*/
    private static void confidencePersistence() {
        header("置信度的持续性:偶尔糊一帧没关系,偶尔清楚一帧不算数");
        Sim s = new Sim();
        BallTracker.Track t = null;
        for (int i = 0; i < 8; i++) t = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.90));
        check("准备:一直很清楚的球,可以追", t != null);
        t = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.40));//忽然糊了一帧
        check("糊了一帧:还在追", t != null);
        check("打分用的置信度是平滑的(一直是0.90,这次看到0.40,记成0.65)", t != null && Math.abs(t.conf - 0.65) < 1e-9);
        t = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.40));
        t = s.step(50, s.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.40));
        check("连着糊下去:就不算了", t == null);

        Sim b = new Sim();
        BallTracker.Track blurry = null;
        for (int i = 0; i < 8; i++) blurry = b.step(50, b.ball(BallTarget.Kind.ALLY_BIG, 0, 30, 0.50));
        check("一直很模糊的球(置信度0.50):不追", blurry == null);
        blurry = b.step(50, b.ball(BallTarget.Kind.ALLY_BIG, 0, 30, 0.99));//忽然有一帧特别清楚
        check("就清楚这一帧:还是不追(这时平滑值已经被抬到0.75了,但不能听它的)", blurry == null);

        Sim h = new Sim();
        BallTracker.Track hover = null;
        for (int i = 0; i < 20; i++)
            hover = h.step(50, h.ball(BallTarget.Kind.ALLY_BIG, 0, 100, (i % 2 == 0) ? 0.90 : 0.50));
        check("置信度在门槛上上上下下(0.9/0.5交替):始终不追", hover == null);

        Sim y = new Sim();
        BallTracker.Track young = null;
        for (int i = 0; i < 3; i++) young = y.step(50, y.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.90));
        young = y.step(50, y.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.50));//第4帧糊了
        check("刚出现的球要求帧帧看清:糊了一帧,先不追", young == null);
        for (int i = 0; i < 3; i++) young = y.step(50, y.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.90));
        check("接着清楚3帧还不够(最近7帧里有1帧是糊的)", young == null);
        young = y.step(50, y.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.90));
        check("清楚满4帧(最近8帧里有7帧清楚):又能追了", young != null);

        Sim o = new Sim();
        BallTracker.Track back = null;
        for (int i = 0; i < 8; i++) back = o.step(50, o.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.90));
        o.now += 400;//消失了一会儿(航迹还在记忆里)
        for (int i = 0; i < 4; i++) back = o.step(50, o.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.40));
        check("消失400ms后重新出现,连着4帧都是糊的:不追(上一回那段“看清”的记录已经作废)", back == null);
        for (int i = 0; i < 8; i++) back = o.step(50, o.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.40));
        check("一直糊下去:始终不追", back == null);
        for (int i = 0; i < 6; i++) back = o.step(50, o.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.90));
        check("重新看清楚到第6帧(最近8帧里只有6帧清楚):还不追", back == null);
        back = o.step(50, o.ball(BallTarget.Kind.ALLY_BIG, 0, 100, 0.90));
        check("第7帧(最近8帧里有7帧清楚):又能追了", back != null);
    }

    /**相机不在吸入口正中心:把"相机看到的"换算成"相对吸入口中心的"。
     * 这里把偏移直接当参数传进去,所以能试各种安装位置(参数墙里的默认值一直是 0)。
     * 轴向:camAheadCm = 相机在吸入口中心前面多少厘米;camLeftCm = 相机在吸入口中心左边多少厘米*/
    private static void cameraOffsetGeometry() {
        header("相机偏移:相机看到的 → 相对吸入口中心的");
        check("偏移是 0:瞄准角原样等于相机偏角", IntakeSelector.aimErrDeg(20, 100, 0, 0) == 20
                && IntakeSelector.aimErrDeg(-37.5, 55, 0, 0) == -37.5 && IntakeSelector.aimErrDeg(0, 35, 0, 0) == 0);
        check("偏移是 0:吸程原样等于相机测到的距离", IntakeSelector.reachCm(20, 100, 0, 0) == 100
                && IntakeSelector.reachCm(-37.5, 55, 0, 0) == 55);

        double left = IntakeSelector.aimErrDeg(0, 100, 0, 10), right = IntakeSelector.aimErrDeg(0, 100, 0, -10);
        check("相机在吸入口中心左边 10 厘米(y=+10),球正对相机:球其实在吸入口中心的左前方 → 要往左转",
                left < -5 && left > -6 && IntakeSelector.reachCm(0, 100, 0, 10) > 100);
        check("相机在右边 10 厘米(y=-10):要往右转,角度大小一样", right > 5 && right < 6 && Math.abs(left + right) < 1e-9);
        check("相机在吸入口中心前面 20 厘米(x=+20):球离吸入口中心 = 100 + 20", IntakeSelector.reachCm(0, 100, 20, 0) == 120);
        check("相机在后面 20 厘米(x=-20):球离吸入口中心 = 100 - 20", IntakeSelector.reachCm(0, 100, -20, 0) == 80);

        double[] p = IntakeSelector.ballFromIntake(36.86989764584402, 100, 0, 10);//3-4-5:球在相机右前方 60 厘米、前方 80 厘米
        check("横向位置要扣掉相机自己的左右偏移:相机在左边 10,所以球在吸入口中心右边 50",
                Math.abs(p[0] - 80) < 1e-6 && Math.abs(p[1] - 50) < 1e-6);

        double tx = Math.toDegrees(Math.atan2(10, 100));
        check("球虽然在相机的右边,但正好在吸入口中心的正前方:不用转",
                Math.abs(IntakeSelector.aimErrDeg(tx, Math.hypot(100, 10), 0, 10)) < 1e-9);
        check("球已经过了吸入口中心这个深度:没有\"往哪边转\"可说(不许算出 180 度让它原地打转)",
                IntakeSelector.aimErrDeg(0, 20, -30, 0) == 0 && IntakeSelector.reachCm(0, 20, -30, 0) == 10);
    }

    /*================= 状态机场景:用"假机器人"整段整段地跑 =================*/

    /**假机器人再加上状态机:每走一步,喂一帧、给传感器读数,状态机就给出这一步的指令*/
    private static class Robot extends Sim {
        final IntakeFsm fsm = new IntakeFsm(now);
        double remainS = 100;//离时段结束还很久(除非某个场景特意改它)

        /**走一步。connected=相机通不通,secured=吸到了吗,enemy=吸到的是敌球吗*/
        void go(double dtMs, boolean connected, boolean secured, boolean enemy, BallTarget... detections) {
            now += (long) dtMs;
            tracker.observe(Arrays.asList(detections), now);
            fsm.step(tracker, yaw, remainS, connected, now, secured, enemy);
        }

        /**最常见的走法:相机通着、没吸到、看见一颗 txDeg 处(右正)、distCm 远的友军大球*/
        void see(double dtMs, double txDeg, double distCm) {
            go(dtMs, true, false, false, ball(BallTarget.Kind.ALLY_BIG, txDeg, distCm, 0.95));
        }

        /**同上,但指定置信度(see()用的0.95是"一直很清楚")*/
        void seeConf(double dtMs, double txDeg, double distCm, double conf) {
            go(dtMs, true, false, false, ball(BallTarget.Kind.ALLY_BIG, txDeg, distCm, conf));
        }
    }

    private static boolean in(Robot r, IntakeFsm.State s) { return r.fsm.state() == s; }

    private static boolean allZero(Robot r) {
        return r.fsm.fwd == 0 && r.fsm.strafe == 0 && r.fsm.turn == 0 && r.fsm.intakePwr == 0;
    }

    /**把假机器人推到"正在APPROACH、球在正前方 distCm 处"。
     * 每帧距离最多变25cm——真实的机器人一帧开不了50cm;变太快的话跟踪器会把球当成新出现的另一颗(这是对的)*/
    private static void approachTo(Robot r, double distCm) {
        r.see(50, 0, 150); r.see(50, 0, 150); r.see(50, 0, 150);
        double d = 150;
        while (d - 25 > distCm) {
            d -= 25;
            r.see(50, 0, d);
        }
        r.see(50, 0, distCm);
    }

    /**再往前一步:球到了吸程(35cm)。调用之后状态机刚进入INTAKE、还没开吸。返回这颗球的航迹编号*/
    private static int approachToIntake(Robot r) {
        approachTo(r, 40);
        r.see(50, 0, 35);
        return r.fsm.holdId();
    }

    /**一颗球的完整旅程:扫描 → 发现 → 接近 → 开吸 → 吸到*/
    private static void fsmNormalPickup() {
        header("状态机:正常吃到一颗球");
        Robot r = new Robot();
        r.go(50, true, false, false);
        check("开场是SEARCH,原地不动", in(r, IntakeFsm.State.SEARCH) && allZero(r));
        r.go(IntakeFsm.SEARCH_DWELL_MS, true, false, false);
        check("原地看完 SEARCH_DWELL_MS 之后才开始转着找", in(r, IntakeFsm.State.SEARCH)
                && r.fsm.turn == IntakeFsm.SEARCH_SPIN_POWER && r.fsm.turn > 0 && r.fsm.fwd == 0);

        r.see(50, 0, 150); r.see(50, 0, 150); r.see(50, 0, 150);
        check("球才出现3帧(100ms):还没确认,继续找", in(r, IntakeFsm.State.SEARCH));
        r.see(50, 0, 125);
        check("连续看到150ms、确认了:进入APPROACH", in(r, IntakeFsm.State.APPROACH));
        r.see(50, 0, 125);
        double approachPower = r.fsm.fwd;
        check("APPROACH:朝球开过去,吸球电机不开", in(r, IntakeFsm.State.APPROACH) && approachPower > 0
                && r.fsm.intakePwr == 0);

        r.see(50, 0, 100); r.see(50, 0, 75); r.see(50, 0, 50);
        check("球一路接近:一直在APPROACH", in(r, IntakeFsm.State.APPROACH));
        r.see(50, 0, 35);
        check("球到了35cm:进入INTAKE", in(r, IntakeFsm.State.INTAKE));
        check("要拱的时间按进INTAKE时的距离算(35cm)", r.fsm.creepMs() == IntakeFsm.creepTimeFor(35));
        r.see(50, 0, 30);
        check("INTAKE:吸球电机开、慢慢往前拱——而不是沿用刚才的接近功率(v1.0在这个分支根本没下发指令)",
                r.fsm.intakePwr == IntakeFsm.INTAKE_POWER && r.fsm.fwd == IntakeFsm.CREEP_POWER
                && approachPower > IntakeFsm.CREEP_POWER);

        r.go(50, true, true, false);//仓位传感器:吸到了
        check("吸到了:立刻停吸、停车,回SEARCH重新找", in(r, IntakeFsm.State.SEARCH) && allZero(r));
        check("SEARCH里不持有任何目标(刚吃掉的那颗也算放下)", r.fsm.holdId() == -1);
        check("吃完之后要等的那一会儿:不短于航迹的记忆期(否则会凭记忆再选一次刚吃掉的球),也不超过SEARCH的原地观察时间",
                IntakeFsm.eatenHoldMs() >= BallTracker.LOST_MS && IntakeFsm.eatenHoldMs() <= IntakeFsm.SEARCH_DWELL_MS);

        r.see(50, 0, 30);//相机还看得见那颗球(它还在吸入口附近)
        check("刚吃完:先不找下一颗,原地不动——不然会回头再追一次刚吃掉的这颗",
                in(r, IntakeFsm.State.SEARCH) && r.fsm.picked() == null && allZero(r));
        r.go(IntakeFsm.eatenHoldMs() + 100, true, false, false);
        check("等够了:恢复找目标(没目标就转着找)", in(r, IntakeFsm.State.SEARCH) && r.fsm.turn == IntakeFsm.SEARCH_SPIN_POWER && r.fsm.turn > 0);

        Robot g = new Robot();
        approachToIntake(g);
        g.go(50, true, true, false);//吃到
        g.go(100, true, false, false);//100ms后那颗球已经看不见了(进了机器人体内)
        check("吃完之后球看不见了:不会凭着记忆回头去追它",
                in(g, IntakeFsm.State.SEARCH) && g.fsm.picked() == null && allZero(g));
    }

    /**防线3:吸进口发现是敌球 → 吐出 → 退开 → 拉黑这个方位 + 这颗球记仇*/
    private static void fsmEjectsEnemy() {
        header("状态机:吸到敌球 → 吐出、退开、拉黑、记仇");
        Robot r = new Robot();
        int id = approachToIntake(r);
        check("准备:已进入INTAKE", in(r, IntakeFsm.State.INTAKE));

        r.go(50, true, true, true);//吸到了,颜色复核:是敌球
        check("进入EJECT", in(r, IntakeFsm.State.EJECT) && allZero(r));
        r.go(50, true, false, false);
        check("EJECT前半段:吸球电机反转吐球,车不动", r.fsm.intakePwr == -IntakeFsm.INTAKE_POWER && r.fsm.fwd == 0);
        r.go(IntakeFsm.EJECT_MS, true, false, false);
        check("吐完:吸球电机停,车后退躲开", in(r, IntakeFsm.State.EJECT) && r.fsm.intakePwr == 0
                && r.fsm.fwd == -IntakeFsm.BACKOFF_POWER);
        r.go(IntakeFsm.BACKOFF_MS, true, false, false);
        check("退开之后回SEARCH", in(r, IntakeFsm.State.SEARCH) && allZero(r));
        check("吐球的方位被拉黑", r.tracker.isBanned(0, r.now));
        check("这颗球被记仇", r.tracker.find(id) != null && r.tracker.find(id).everEnemy);

        for (int i = 0; i < 6; i++) r.see(50, 0, 60);//机器人转回来又看见它,视觉照样说"友军"
        check("视觉还是说友军:不追", in(r, IntakeFsm.State.SEARCH) && r.fsm.picked() == null);
        r.now += BallTracker.BAN_FIRST_MS + 1;
        r.see(50, 0, 60);
        check("拉黑2秒过期之后:还是不追(记仇还在)", in(r, IntakeFsm.State.SEARCH) && r.fsm.picked() == null);
    }

    /**拱到位了却没吸到(球歪了/根本不在):退开,这个方位先缓2秒,免得在同一颗球上反复撞*/
    private static void fsmIntakeTimeout() {
        header("状态机:拱完了还没吸到 → 退开,这个方向先缓缓");
        Robot r = new Robot();
        approachToIntake(r);
        long creepMs = r.fsm.creepMs();
        check("拱的时间由距离算出来,不是写死的:35cm→" + creepMs + "ms,60cm更久,再远也有上限兜着",
                creepMs == IntakeFsm.creepTimeFor(35) && IntakeFsm.creepTimeFor(60) > creepMs
                && IntakeFsm.creepTimeFor(500) == IntakeFsm.INTAKE_MAX_MS);
        r.go(creepMs - 1, true, false, false);
        check("还差1ms到时限:还在吸", in(r, IntakeFsm.State.INTAKE) && r.fsm.intakePwr == IntakeFsm.INTAKE_POWER);
        r.go(1, true, false, false);
        check("到了时限还没吸到:进BACKOFF,吸球电机停", in(r, IntakeFsm.State.BACKOFF) && allZero(r));
        r.go(50, true, false, false);
        check("BACKOFF:后退", r.fsm.fwd == -IntakeFsm.BACKOFF_POWER && r.fsm.intakePwr == 0);
        check("没吸到的方位被短暂拉黑", r.tracker.isBanned(0, r.now));
        r.go(IntakeFsm.BACKOFF_MS, true, false, false);
        check("退开之后回SEARCH", in(r, IntakeFsm.State.SEARCH));
        for (int i = 0; i < 8; i++) r.see(50, 0, 70);//球还在那儿,相机也看得见
        check("拉黑的2秒内:不再去撞同一颗球", in(r, IntakeFsm.State.SEARCH) && r.fsm.picked() == null);
    }

    /**设计稿§2.1:画面边缘的目标不直接追,先原地转过去、到画面中心看清了再说;带冷却和末段禁用*/
    private static void fsmEdgeTarget() {
        header("状态机:画面边缘的球 → 先转过去看清(VERIFY_SPIN)");
        Robot r = new Robot();
        double bearing = -45;//球在场地系-45°。开场车头朝0°,所以它在画面右边缘45°处
        r.see(50, 45, 120); r.see(50, 45, 120); r.see(50, 45, 120);
        check("还没确认:还在扫描,不追", in(r, IntakeFsm.State.SEARCH) && r.fsm.fwd == 0);
        r.see(50, 45, 120);
        check("确认了:不直接追,进VERIFY_SPIN", in(r, IntakeFsm.State.VERIFY_SPIN));
        r.see(50, 45, 120);
        check("原地朝球的方向转(球在右→右转),不前进不平移", r.fsm.turn > 0 && r.fsm.fwd == 0 && r.fsm.strafe == 0);

        for (int i = 0; i < 7; i++) {//机器人往右转:车头每步右转5°(逆时针为正,所以yaw在变小),球在画面里逐渐移向中心
            r.yaw -= 5;
            r.see(50, r.yaw - bearing, 120);
        }
        check("转到球进入画面中心附近:验证通过,进APPROACH", in(r, IntakeFsm.State.APPROACH));
        r.see(50, r.yaw - bearing, 120);
        check("APPROACH:往球的方向开", in(r, IntakeFsm.State.APPROACH) && r.fsm.fwd > 0);

        r.go(400, true, false, false);//这颗球其实是看走眼的"半个球":转到眼前反而消失了
        check("转过去发现目标没了:回SEARCH,并且放下它", in(r, IntakeFsm.State.SEARCH) && r.fsm.holdId() == -1);

        r.yaw = 0;
        for (int i = 0; i < 6; i++) r.see(50, 40, 200);//冷却期内,画面里又冒出一个边缘目标
        check("冷却期(5秒)内:边缘目标既不触发VERIFY_SPIN、也不会被追", in(r, IntakeFsm.State.SEARCH) && r.fsm.picked() == null);
        r.now += IntakeFsm.SPIN_CD_MS;
        for (int i = 0; i < 4; i++) r.see(50, 40, 200);//冷却(5秒)比记忆(3秒)长,所以它早被忘了,要重新确认
        check("冷却结束:边缘目标重新确认后,又可以触发VERIFY_SPIN了", in(r, IntakeFsm.State.VERIFY_SPIN));

        Robot l = new Robot();
        l.remainS = IntakeFsm.SPIN_LOCKOUT_S;
        for (int i = 0; i < 6; i++) l.see(50, 40, 120);
        check("最后 SPIN_LOCKOUT_S 秒:不再VERIFY_SPIN", in(l, IntakeFsm.State.SEARCH));

        //验证的"验"字:转到位还不够,转到位的那一刻必须确实还看得见它
        Robot g = new Robot();
        for (int i = 0; i < 4; i++) g.see(50, 45, 120);
        check("准备:在VERIFY_SPIN", in(g, IntakeFsm.State.VERIFY_SPIN));
        g.yaw = -20;
        g.see(50, g.yaw + 45, 120);//往右转了20°:还看得见
        g.yaw = -36;
        g.go(50, true, false, false);//又转了16°,已经对准了(只偏9°),但这一帧没看到它
        check("对准了、可是没亲眼看到它:不算验证通过,不开过去", in(g, IntakeFsm.State.VERIFY_SPIN) && g.fsm.fwd == 0);
        g.go(350, true, false, false);
        check("之后再没看到:回SEARCH", in(g, IntakeFsm.State.SEARCH));

        //追的过程中车头被撞偏了:球从正前方跑到了画面边缘
        Robot d = new Robot();
        approachTo(d, 60);
        d.yaw = -40;
        d.see(50, -40, 60);
        check("追着追着车头偏出去40°:暂停接近,进VERIFY_SPIN转回去", in(d, IntakeFsm.State.VERIFY_SPIN));
    }

    /**盲吸:默认关。打开之后,也只对"在很近、且对准了的位置看不见"的球生效;因身份存疑而出局的球,开关打开也不吸*/
    private static void fsmBlindIntake() {
        header("状态机:盲吸(默认关;打开后也只对'太近看不见'的球生效)");
        // 模型在贴得很近时反而认不出球:球在60cm处、正前方,追着追着突然看不见了
        Robot r = new Robot();
        approachTo(r, 60);
        check("准备:在APPROACH,球在60cm", in(r, IntakeFsm.State.APPROACH));
        r.go(400, true, false, false);
        check("开关关着:当作目标丢失,回SEARCH,吸球电机从没开过", in(r, IntakeFsm.State.SEARCH) && r.fsm.intakePwr == 0);

        IntakeFsm.BLIND_INTAKE = true;
        try {
            Robot b = new Robot();
            approachTo(b, 60);
            b.go(400, true, false, false);
            check("开关打开:球是在很近的地方看不见的 → 盲吸", in(b, IntakeFsm.State.INTAKE));
            check("盲吸要拱的时间按最后看到的距离(60cm)算", b.fsm.creepMs() == IntakeFsm.creepTimeFor(60));
            b.go(50, true, false, false);
            check("盲吸中:吸球电机开、慢慢拱", b.fsm.intakePwr == IntakeFsm.INTAKE_POWER && b.fsm.fwd == IntakeFsm.CREEP_POWER);

            Robot far = new Robot();
            approachTo(far, 130);
            far.go(400, true, false, false);
            check("球是在130cm处看不见的:那不叫'太近',不盲吸", in(far, IntakeFsm.State.SEARCH) && far.fsm.intakePwr == 0);

            Robot off = new Robot();
            approachTo(off, 60);
            off.see(50, 8, 60); off.see(50, 16, 60);
            off.go(400, true, false, false);
            check("球最后是在没对准的位置(偏16°)看不见的:不盲吸", in(off, IntakeFsm.State.SEARCH) && off.fsm.intakePwr == 0);

            Robot v = new Robot();
            approachTo(v, 60);
            v.go(50, true, false, false, v.ball(BallTarget.Kind.ENEMY_BIG, 0, 60, 0.95));//有一帧像敌球 → 这颗球出局
            check("这颗球有一帧像敌球:立刻出局回SEARCH,而且不再持有它",
                    in(v, IntakeFsm.State.SEARCH) && v.fsm.holdId() == -1);
            v.go(400, true, false, false);
            check("之后它贴着眼前看不见了:开关打开也不盲吸", in(v, IntakeFsm.State.SEARCH) && v.fsm.intakePwr == 0);
        } finally {
            IntakeFsm.BLIND_INTAKE = false;//开关还原,别影响后面的场景
        }
    }

    /**几条兜底:相机断开、太久没进展、时间到——任何一个触发都必须全停,而且停了就不再动*/
    private static void fsmFailSafes() {
        header("状态机:兜底保护");
        Robot r = new Robot();
        r.go(100, false, false, false);
        check("相机刚断100ms:先不慌", in(r, IntakeFsm.State.SEARCH));
        r.go(IntakeFsm.VISION_DEAD_MS + 1, false, false, false);//刚好到线不算,要超过
        check("断开超过 VISION_DEAD_MS:PARK,全停", in(r, IntakeFsm.State.PARK) && allZero(r));
        r.see(500, 0, 100);
        check("PARK是终点:相机恢复了也不会重新开跑", in(r, IntakeFsm.State.PARK) && allZero(r));

        Robot c = new Robot();
        for (int i = 0; i < 10; i++) {
            c.go(100, false, false, false);
            c.go(100, true, false, false);
        }
        check("断100ms、通100ms反复10次(累计断了1秒,但从没连续断过1.5秒):不停车", in(c, IntakeFsm.State.SEARCH));

        Robot m = new Robot();
        approachToIntake(m);
        m.see(50, 0, 30);
        m.go(50, false, false, false);
        check("吸球途中相机刚断:继续吸(给相机一点恢复时间)", in(m, IntakeFsm.State.INTAKE)
                && m.fsm.intakePwr == IntakeFsm.INTAKE_POWER);
        m.go(IntakeFsm.VISION_DEAD_MS + 1, false, false, false);
        check("断久了:立刻PARK,吸球电机马上停", in(m, IntakeFsm.State.PARK) && allZero(m));

        Robot n = new Robot();
        n.go(IntakeFsm.NO_PROGRESS_MS, true, false, false);
        check("刚好 NO_PROGRESS_MS 没吃到球:还没停", in(n, IntakeFsm.State.SEARCH));
        n.go(1, true, false, false);
        check("超过 NO_PROGRESS_MS:PARK", in(n, IntakeFsm.State.PARK) && allZero(n));

        Robot p = new Robot();
        approachToIntake(p);
        p.go(50, true, true, false);//吃到一颗:计时重新开始
        p.go(IntakeFsm.NO_PROGRESS_MS, true, false, false);
        check("吃到球之后重新计时:再过 NO_PROGRESS_MS 才算没进展(此刻还没到)", in(p, IntakeFsm.State.SEARCH));
        p.go(1, true, false, false);
        check("过了:PARK", in(p, IntakeFsm.State.PARK));

        Robot e = new Robot();
        e.remainS = IntakeFsm.ENDGAME_PARK_S;
        e.go(1, true, false, false);
        check("离结束还有 ENDGAME_PARK_S 整:继续跑", in(e, IntakeFsm.State.SEARCH));
        e.remainS = IntakeFsm.ENDGAME_PARK_S - 0.1;
        e.go(1, true, false, false);
        check("进入最后 ENDGAME_PARK_S 秒:PARK", in(e, IntakeFsm.State.PARK) && allZero(e));
        check("末段花粉加权:最后10秒之外是1.0,之内是1.5",
                IntakeFsm.pollenBoost(IntakeFsm.ENDGAME_POLLEN_S) == 1.0
                && IntakeFsm.pollenBoost(IntakeFsm.ENDGAME_POLLEN_S - 0.1) == IntakeFsm.ENDGAME_POLLEN_BOOST);
    }

    /**压力测试:造一个"几颗球不断靠近、相机偶尔漏看、模型偶尔认错颜色、相机偶尔断线"的随机局面,灌2万步。
     * 不检查它做出了多聪明的决定,只检查两条硬规矩:
     *  输出永远合法;吸球电机只在INTAKE(正转)和EJECT(反转)里才会转。
     * 同时统计七种状态是不是都走到过——没走到的状态等于没测*/
    private static void fsmRandomStress() {
        header("状态机:随机压力测试(2万步)");
        Random rnd = new Random(20261125);//固定种子:每次跑的是同一串随机数,失败了可以复现
        BallTarget.Kind[] kinds = BallTarget.Kind.values();
        EnumSet<IntakeFsm.State> visited = EnumSet.noneOf(IntakeFsm.State.class);
        boolean powerOk = true, motorOk = true;
        int episodes = 0;

        for (int done = 0; done < 20000; ) {
            Robot r = new Robot();
            episodes++;
            final int n = 4;//场上4颗球:各有方位、身份、置信度
            double[] bearing = new double[n], dist = new double[n], conf = new double[n];
            BallTarget.Kind[] kind = new BallTarget.Kind[n];
            for (int k = 0; k < n; k++) newBall(rnd, kinds, bearing, dist, conf, kind, k);

            for (int i = 0; i < 400 && done < 20000; i++, done++) {
                r.yaw += rnd.nextDouble() * 20 - 10;//车头慢慢晃
                BallTarget[] dets = new BallTarget[n];
                int m = 0;
                for (int k = 0; k < n; k++) {
                    dist[k] -= rnd.nextDouble() * 6;//球在靠近
                    conf[k] = Math.max(0.3, Math.min(1.0, conf[k] + rnd.nextDouble() * 0.2 - 0.1));
                    if (dist[k] < 15) {//到脚边了:换一颗新的远球
                        newBall(rnd, kinds, bearing, dist, conf, kind, k);
                        continue;
                    }
                    if (rnd.nextDouble() < 0.08) continue;//这一帧漏看了
                    double tx = r.yaw - bearing[k];
                    if (Math.abs(tx) > 47) continue;//不在视野里
                    //偶尔认错颜色
                    BallTarget.Kind seenAs = (rnd.nextDouble() < 0.03) ? kinds[rnd.nextInt(kinds.length)] : kind[k];
                    dets[m++] = r.ball(seenAs, tx, dist[k], conf[k]);
                }
                r.remainS = 3 + rnd.nextDouble() * 60;//先避开最后3秒,让各种状态都有机会走到
                r.go(10 + rnd.nextInt(300), rnd.nextInt(10) != 0, rnd.nextInt(7) == 0, rnd.nextInt(3) == 0,
                        Arrays.copyOf(dets, m));

                double[] p = {r.fsm.fwd, r.fsm.strafe, r.fsm.turn, r.fsm.intakePwr};
                for (double v : p) powerOk &= !Double.isNaN(v) && Math.abs(v) <= 1.0;

                IntakeFsm.State s = r.fsm.state();
                visited.add(s);
                if (s != IntakeFsm.State.INTAKE && s != IntakeFsm.State.EJECT && r.fsm.intakePwr != 0) motorOk = false;
                if (s == IntakeFsm.State.INTAKE && r.fsm.intakePwr != 0 && r.fsm.intakePwr != IntakeFsm.INTAKE_POWER) motorOk = false;
                if (s == IntakeFsm.State.EJECT && r.fsm.intakePwr != 0 && r.fsm.intakePwr != -IntakeFsm.INTAKE_POWER) motorOk = false;
                if (s == IntakeFsm.State.PARK && !allZero(r)) motorOk = false;
            }
        }
        check("每一步四个功率都是 -1~1 之间的有限数", powerOk);
        check("吸球电机只在INTAKE(正转)和EJECT(反转)里转;PARK里四个功率全零", motorOk);
        check("七种状态全部被走到过(" + visited.size() + "/7,共" + episodes + "局)",
                visited.size() == IntakeFsm.State.values().length);
    }

    private static void newBall(Random rnd, BallTarget.Kind[] kinds, double[] bearing, double[] dist,
                                double[] conf, BallTarget.Kind[] kind, int k) {
        bearing[k] = rnd.nextDouble() * 360 - 180;
        dist[k] = 150 + rnd.nextDouble() * 150;
        conf[k] = 0.5 + rnd.nextDouble() * 0.5;
        kind[k] = kinds[rnd.nextInt(kinds.length)];
    }

    /**开吸是整个流程里最不可逆的一步,所以它的每一道门都要有测试守着:
     * 只认亲眼看到的、要对准、要持续看清;盲吸只对"看不见了"的球生效,不对"被挡路"的球生效*/
    private static void fsmIntakeGates() {
        header("状态机:开吸的几道门");

        Robot m = new Robot();
        approachTo(m, 50);
        m.see(50, 8, 45); m.see(50, 16, 40); m.see(50, 22, 30);//球到了30cm,但偏了22°:还没对准
        check("球到了30cm、但偏了22°:先不开吸,继续对准", in(m, IntakeFsm.State.APPROACH) && m.fsm.intakePwr == 0);
        for (int i = 0; i < 3; i++) {//这时相机认不出它了(贴得太近);车头还在朝它转
            m.yaw -= 5;
            m.go(50, true, false, false);
        }
        check("凭记忆对准了不算数:这一帧没亲眼看到,不开吸", in(m, IntakeFsm.State.APPROACH) && m.fsm.intakePwr == 0);
        m.go(200, true, false, false);
        check("最后还是丢了:回SEARCH,全程没开过吸", in(m, IntakeFsm.State.SEARCH) && m.fsm.intakePwr == 0);

        Robot k = new Robot();
        for (int i = 0; i < 10; i++) k.seeConf(50, 0, 30, 0.50);
        check("球贴在吸程里(30cm、正前方),但一直看不清:不开吸", in(k, IntakeFsm.State.SEARCH) && k.fsm.intakePwr == 0);
        k.seeConf(50, 0, 30, 0.99);
        check("忽然有一帧特别清楚:仍然不开吸(宁漏勿错)", in(k, IntakeFsm.State.SEARCH) && k.fsm.intakePwr == 0);
        for (int i = 0; i < 5; i++) k.seeConf(50, 0, 30, 0.99);
        check("清楚到第6帧还不够(最近8帧里只有6帧清楚)", in(k, IntakeFsm.State.SEARCH) && k.fsm.intakePwr == 0);
        k.seeConf(50, 0, 30, 0.99);
        check("清楚到第7帧(最近8帧里有7帧清楚):算看清了,可以追 → 进APPROACH", in(k, IntakeFsm.State.APPROACH));
        k.seeConf(50, 0, 30, 0.99);
        check("再走一步,APPROACH发现它在吸程里、又对准了 → 开吸", in(k, IntakeFsm.State.INTAKE));

        IntakeFsm.BLIND_INTAKE = true;
        try {
            Robot blk = new Robot();
            approachTo(blk, 40);
            boolean everIntake = false, motorEverOn = false;
            for (int i = 0; i < 8; i++) {//它前面冒出一颗更近的敌球
                blk.go(50, true, false, false, blk.ball(BallTarget.Kind.ALLY_BIG, 0, 40, 0.95),
                        blk.ball(BallTarget.Kind.ENEMY_BIG, 0, 20, 0.95));
                everIntake |= in(blk, IntakeFsm.State.INTAKE);
                motorEverOn |= blk.fsm.intakePwr != 0;
            }
            check("友军球就在眼前,但前面挡着一颗敌球:不选它", in(blk, IntakeFsm.State.SEARCH) && blk.fsm.picked() == null);
            check("盲吸开关开着也不吸:这颗球是被挡路才出局的,它明明还看得见", !everIntake && !motorEverOn);
        } finally {
            IntakeFsm.BLIND_INTAKE = false;
        }
    }

    /**“吸到了一颗”认的是传感器从无到有的那一瞬间,不是“它现在是亮的”。
     * 仓里已经有球压着传感器的时候,它开吸之前就亮着——如果把这当成“又吃到一颗”,
     * 机器人会每颗球都只是开到跟前、点个名就走,一颗也吸不进来*/
    private static void fsmArrivalEdge() {
        header("状态机:“吸到了”认的是传感器从无到有的那一瞬间");
        Robot s = new Robot();
        approachTo(s, 40);
        s.go(50, true, true, false, s.ball(BallTarget.Kind.ALLY_BIG, 0, 35, 0.95));//到位;这时传感器已经是亮的
        check("准备:进入INTAKE", in(s, IntakeFsm.State.INTAKE));
        s.go(50, true, true, false);
        check("开吸之后传感器一直亮着(没有从无到有):不算吃到,照常吸、照常往前拱",
                in(s, IntakeFsm.State.INTAKE) && s.fsm.intakePwr == IntakeFsm.INTAKE_POWER
                && s.fsm.fwd == IntakeFsm.CREEP_POWER);
        s.go(IntakeFsm.creepTimeFor(35), true, true, false);
        check("拱到头也没等到“从无到有”:按没吸到处理,退开(球其实吸进去了,只是传感器没说出来)",
                in(s, IntakeFsm.State.BACKOFF));

        Robot e = new Robot();
        approachTo(e, 40);
        e.go(50, true, true, false, e.ball(BallTarget.Kind.ALLY_BIG, 0, 35, 0.95));
        e.go(50, true, true, true);
        check("敌球也一样:传感器没有从无到有,就算颜色复核说是敌球也不触发吐球", in(e, IntakeFsm.State.INTAKE));
        e.go(50, true, false, false);//传感器灭了一下
        e.go(50, true, true, true);//又亮了:真的有一颗球进来了
        check("灭一下再亮:这才算吸到一颗;是敌球就吐", in(e, IntakeFsm.State.EJECT));
    }

    /**把参数墙里的偏移临时改成非 0,看状态机是不是真的按"吸入口中心"在做决定。
     * 只测默认的 0 等于什么都没测:0 的时候新旧算法的结果本来就一样*/
    private static void fsmCameraOffset() {
        header("状态机:相机不在吸入口正中心");
        double x0 = IntakeSelector.CAM_OFFSET_X_CM, y0 = IntakeSelector.CAM_OFFSET_Y_CM;
        try {
            IntakeSelector.CAM_OFFSET_X_CM = 20;   //相机装在吸入口中心前面 20 厘米
            IntakeSelector.CAM_OFFSET_Y_CM = 0;
            Robot ahead = new Robot();
            approachTo(ahead, 40);
            ahead.see(50, 0, 30);//离相机 30 厘米。相机偏移是 0 的话,这里已经该开吸了
            check("相机在吸入口前面 20 厘米:球离相机 30 厘米,但离吸入口中心还有 50 厘米 → 先不开吸",
                    in(ahead, IntakeFsm.State.APPROACH) && ahead.fsm.intakePwr == 0);
            ahead.see(50, 0, 15);//离吸入口中心 35 厘米
            check("球离吸入口中心 35 厘米了(离相机只有 15):进 INTAKE,拱的时间按 35 厘米算",
                    in(ahead, IntakeFsm.State.INTAKE) && ahead.fsm.creepMs() == IntakeFsm.creepTimeFor(35));

            IntakeSelector.CAM_OFFSET_X_CM = -20;  //相机装在吸入口中心后面 20 厘米
            Robot behind = new Robot();
            approachTo(behind, 60);
            behind.see(50, 0, 56);//离吸入口中心 36 厘米,还差一点
            check("相机在吸入口后面 20 厘米:球离相机 56 厘米 = 离吸入口中心 36 厘米 → 还没到", in(behind, IntakeFsm.State.APPROACH));
            behind.see(50, 0, 55);//离吸入口中心 35 厘米
            check("球离相机还有 55 厘米,但离吸入口中心已经只有 35 厘米 → 开吸", in(behind, IntakeFsm.State.INTAKE));

            IntakeSelector.CAM_OFFSET_X_CM = 0;
            IntakeSelector.CAM_OFFSET_Y_CM = 10;   //相机装在吸入口中心左边 10 厘米
            Robot left = new Robot();
            approachTo(left, 100);
            left.see(50, 0, 100);//球正对相机
            check("相机在吸入口左边:球正对相机时,车头要往左转一点,吸入口才对得准(车头转 < 0,横移 < 0)",
                    in(left, IntakeFsm.State.APPROACH) && left.fsm.turn < 0 && left.fsm.strafe < 0 && left.fsm.fwd > 0);
            IntakeSelector.CAM_OFFSET_Y_CM = -10;  //换到右边
            Robot right = new Robot();
            approachTo(right, 100);
            right.see(50, 0, 100);
            check("相机在吸入口右边:往右转(车头转 > 0,横移 > 0)",
                    in(right, IntakeFsm.State.APPROACH) && right.fsm.turn > 0 && right.fsm.strafe > 0);

            // 贴得很近、又偏得很多:按吸入口中心算已经偏了 45 度,但相机看到的仍然是正前方(没有偏到画面边缘)
            Robot near = new Robot();
            approachTo(near, 40);
            near.see(50, 0, 20);
            near.see(50, 0, 10);
            check("相机看到的角没到画面边缘:就算按吸入口中心算偏了 45 度,也还在 APPROACH 里调,不算'偏到边缘'",
                    in(near, IntakeFsm.State.APPROACH) && near.fsm.turn > 0);
        } finally {
            IntakeSelector.CAM_OFFSET_X_CM = x0;
            IntakeSelector.CAM_OFFSET_Y_CM = y0;
        }
        Robot back = new Robot();
        approachTo(back, 40);
        back.see(50, 0, 34);//不到 35 厘米:偏移是 0 的时候,这就该开吸了
        check("偏移改回 0 之后:行为和原来一模一样(球离相机 34 厘米就开吸)", IntakeSelector.CAM_OFFSET_X_CM == 0
                && IntakeSelector.CAM_OFFSET_Y_CM == 0 && in(back, IntakeFsm.State.INTAKE));
    }

    /*================= 工具 =================*/

    /**一个极简的"假机器人":有朝向和时钟,每步喂一帧给跟踪器,再问选球器选谁*/
    private static class Sim {
        final BallTracker tracker = new BallTracker();
        long now = 10000;
        double yaw = 0;

        /**造一个检测:球在画面的 txDeg 处(右正),距离 distCm。场地方位和Limelight封装调的是同一个函数*/
        BallTarget ball(BallTarget.Kind kind, double txDeg, double distCm, double conf) {
            double ta = 3600.0 / (distCm * distCm);//假装 K=60:只要 ta 和距离自洽就行
            return new BallTarget(kind, 0, txDeg, BallTracker.bearingOf(yaw, txDeg), ta, distCm, conf);
        }

        /**前进 dtMs,喂一帧(可能什么都没有),不带"正在追的目标"去选*/
        BallTracker.Track step(double dtMs, BallTarget... detections) {
            return selectAfter(dtMs, -1, detections);
        }

        /**同上,但告诉选球器"我正在追 heldId"——用来测滞回*/
        BallTracker.Track stepHeld(int heldId, double dtMs, BallTarget... detections) {
            return selectAfter(dtMs, heldId, detections);
        }

        private BallTracker.Track selectAfter(double dtMs, int heldId, BallTarget... detections) {
            now += (long) dtMs;
            List<BallTarget> frame = new ArrayList<>(Arrays.asList(detections));
            tracker.observe(frame, now);
            return IntakeSelector.select(tracker, yaw, 1.0, now, heldId, false);
        }
    }

    private static void header(String title) {
        System.out.printf("%n【%s】%n", title);
    }

    private static void check(String what, boolean ok) {
        if (ok) passed++; else failed++;
        System.out.printf("  %s %s%n", ok ? "√" : "× 失败!", what);
    }
}
