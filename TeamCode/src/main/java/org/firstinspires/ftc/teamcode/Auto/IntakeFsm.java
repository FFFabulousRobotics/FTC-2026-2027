package org.firstinspires.ftc.teamcode.Auto;

/**拾球状态机,也就是整套算法的"大脑"。纯逻辑:不碰任何硬件,时间全部由调用方传入,
 * 所以能在电脑上整段整段地跑(Test/OfflineTest 里的"状态机"场景)。
 * 输入:球跟踪器 + 几个传感器读数。输出:当前状态 + 四个功率(前进/右移/右转/吸球电机)。
 * AutoIntake 只剩两件事:把相机数据喂给 BallTracker,把这里的输出写给电机。
 *
 * 两条写法上的规矩,都是v1.0踩过的坑:
 *  1. step() 每被调用一次,四个输出都会被重新赋值——不存在"某个分支忘了下发、电机沿用上一轮"的情况
 *  2. 换状态只走 enter()——同一步里换了状态,这一步的输出就保持全零,新状态的动作从下一步开始*/
public class IntakeFsm {

    /*-------- 参数墙:状态机与自救(设计稿§3) --------
     * 评分/挡路/接近控制 在 IntakeSelector;确认/丢失/拉黑 在 BallTracker*/
    public static final long   SEARCH_DWELL_MS   = 600;  //开始扫描前先原地看这么久(目标可能只是被挡了一下)
    public static final double SEARCH_SPIN_POWER = 0.25; //扫描时的转向功率(往右转)
    public static final double SPIN_TOL_DEG      = 12;   //VERIFY_SPIN:转到离车头这么近就算转到位
    public static final long   SPIN_CD_MS        = 5000; //两次 VERIFY_SPIN 的最小间隔(防陀螺式打转)
    public static final long   SPIN_MAX_MS       = 1500; //一次 VERIFY_SPIN 最多转多久,超时放弃
    public static final double SPIN_LOCKOUT_S    = 8;    //最后这么多个秒不再 VERIFY_SPIN(没时间折腾)

    public static final double INTAKE_RANGE_CM   = 35;   //球进入这个距离、且对准了 → 开吸
    public static final double INTAKE_TOL_DEG    = 12;   //"对准了" = 偏离车头不超过这个角
    public static final double INTAKE_POWER      = 1.0;  //正 = 吸,负 = 吐。吸球机构交给队友的 Intake 类去动,功率大小由它定,所以这里只有正负号起作用
    public static final double CREEP_POWER       = 0.2;  //开吸时往前拱的功率
    public static final double CREEP_CM_PER_S    = 20;   //TODO: 台架实测:CREEP_POWER功率下每秒前进多少cm。开吸后拱多久全靠它算
    public static final double INTAKE_MARGIN_CM  = 10;   //拱的距离 = 进INTAKE时的距离 + 这么多(把球彻底吞进去)
    public static final long   INTAKE_MAX_MS     = 6000; //拱的时间上限:CREEP_CM_PER_S万一填小了,也不至于停不下来
    public static boolean BLIND_INTAKE           = false;//球贴到眼前、突然看不见时,按最后看到的位置盲吸一下。
                                                         //这是全程序唯一一处"没亲眼看到也开吸",防线3(颜色复核)接通之前保持false
    public static final double BLIND_RANGE_CM    = 80;   //盲吸的判定:球是在这么近的地方看不见的,才算"太近了反而认不出"
    public static final long   AFTER_EAT_HOLD_MS  = 500; //吃到一颗之后先等这么久再找下一颗。机构要把球收进去;
                                                         //而且刚吃掉的那颗球在航迹里还有一小段"记忆期",不能回头再去追它。
                                                         //TODO: 台架上量:吃完一颗到机构能接下一颗,实际要多久

    public static final long   EJECT_MS          = 800;  //吐球时长(真机实测吐干净的最短时间)
    public static final long   BACKOFF_MS        = 600;  //后退时长(吐球后、没吸到后,退开大约一个身位)
    public static final double BACKOFF_POWER     = 0.25; //TODO: 台架上量600ms退了多远

    public static final double PERIOD_S             = 30;   //本时段总时长,按正式手册填
    public static final double ENDGAME_POLLEN_S     = 10;   //最后这么多个秒:花粉加权
    public static final double ENDGAME_POLLEN_BOOST = 1.5;  //加权倍数(设计稿§4.11)
    public static final double ENDGAME_PARK_S       = 3;    //最后这么多个秒:停泊保分
    public static final long   NO_PROGRESS_MS       = 20000;//这么久没吃到一颗球就停。兜底:不管卡在什么循环里,都不会原地打转到比赛结束
    public static final long   VISION_DEAD_MS       = 1500; //相机断开这么久就停

    public enum State { SEARCH, APPROACH, VERIFY_SPIN, INTAKE, BACKOFF, EJECT, PARK }

    /*-------- 本轮输出。step() 每次都会重新赋值,AutoIntake 直接拿去写电机 --------*/
    public double fwd, strafe, turn, intakePwr;

    private State state = State.SEARCH;
    private String note = "开始";//为什么进了当前状态,调试时先看这个
    private int holdId = -1;     //正在追/吸的那颗球(航迹编号);-1 = 没有
    private BallTracker.Track picked;//这一轮选球器选出的目标,没有就是null(给telemetry看)

    private long stateStartMs;               //进入当前状态的时刻
    private long creepMs;                    //这一轮开吸要拱多久(进INTAKE时按距离算好)
    private long lastSpinMs;                 //上一次进 VERIFY_SPIN 的时刻
    private long lastBallMs;                 //上一次吃到球的时刻
    private long visionDeadSinceMs = -1;     //相机从什么时候开始断着;-1 = 没断
    private long eatenHoldUntilMs;           //吃到球之后,到这个时刻为止先不找下一颗
    private boolean securedLast;             //上一步的仓位传感器读数。"吸到了一颗"看的是它从无到有的那一瞬间

    /**@param nowMs 开跑时刻(调用方时钟),之后 step() 传入的 nowMs 都从这之后算起*/
    public IntakeFsm(long nowMs) {
        stateStartMs = nowMs;
        lastSpinMs = nowMs - SPIN_CD_MS;//开场时冷却本来就是好的
        lastBallMs = nowMs;
        eatenHoldUntilMs = nowMs;
    }

    public State state()                     { return state; }
    public String note()                     { return note; }
    public int holdId()                      { return holdId; }
    public BallTracker.Track picked()        { return picked; }
    public long inStateMs(long nowMs)        { return nowMs - stateStartMs; }
    public long creepMs()                    { return creepMs; }

    /**开吸后要拱多久:把"距离 + 余量"那么长的路,用 CREEP_POWER 的速度走完。
     * 由距离算出来,所以把 INTAKE_RANGE_CM 调大调小,时间会自动跟着变*/
    public static long creepTimeFor(double distCm) {
        return (long) Math.min((distCm + INTAKE_MARGIN_CM) / CREEP_CM_PER_S * 1000.0, INTAKE_MAX_MS);
    }

    /**吃到球之后要等多久。
     * 不能短于航迹的记忆期(BallTracker.LOST_MS):刚吃掉的那颗球离得最近、分数最高,
     *   只要它的航迹还没过记忆期,就会被再选一次。
     * 也不必长于 SEARCH 的原地观察时间(SEARCH_DWELL_MS):SEARCH 本来就要先原地看这么久才转着找*/
    public static long eatenHoldMs() {
        return Math.max(BallTracker.LOST_MS, Math.min(AFTER_EAT_HOLD_MS, SEARCH_DWELL_MS));
    }

    /**末段花粉加权:平时1.0,最后 ENDGAME_POLLEN_S 秒变成 ENDGAME_POLLEN_BOOST*/
    public static double pollenBoost(double remainS) {
        return remainS < ENDGAME_POLLEN_S ? ENDGAME_POLLEN_BOOST : 1.0;
    }

    /**走一轮。
     * @param remainS         本时段还剩多少秒
     * @param visionConnected 相机通不通(Limelight.isConnected)
     * @param secured         仓位传感器这一轮的读数。"吸到了一颗"认的是它从无到有的那一瞬间,
     *                        不是"它现在是亮的"——仓里已经有球压着传感器时,它开吸前就亮着,那不算又吃到一颗。
     *                        所以传感器只要如实上报"现在有没有球"就行,是电平型还是脉冲型都可以
     * @param enemyAtMouth    颜色复核:刚吸到的这颗是敌球吗(只在"吸到了"的那一轮有意义)*/
    public void step(BallTracker tracker, double yawDeg, double remainS, boolean visionConnected,
                     long nowMs, boolean secured, boolean enemyAtMouth) {
        fwd = 0; strafe = 0; turn = 0; intakePwr = 0;//规矩1:先归零,下面谁要动谁再赋值
        picked = null;

        boolean arrived = secured && !securedLast;//这一轮有没有一颗球"刚进来"
        securedLast = secured;//每轮都要记,不管在哪个状态

        /*相机断开的计时:通着就清零,断着才累加*/
        if (visionConnected) visionDeadSinceMs = -1;
        else if (visionDeadSinceMs < 0) visionDeadSinceMs = nowMs;
        long deadForMs = (visionDeadSinceMs < 0) ? 0 : nowMs - visionDeadSinceMs;

        /*全局保护:任何一条触发都进 PARK,进了就不再出来*/
        if (state != State.PARK) {
            if (remainS < ENDGAME_PARK_S)          enter(State.PARK, "时间到,停泊保分", nowMs);
            else if (deadForMs > VISION_DEAD_MS)   enter(State.PARK, "相机断开", nowMs);
            else if (nowMs - lastBallMs > NO_PROGRESS_MS) enter(State.PARK, "太久没吃到球", nowMs);
        }

        /*选球。正在吸的那颗球是锁定的,不重选;PARK 里不需要选;刚吃完一颗的那一小会儿也不选。
         * 画面边缘的球,只有在允许 VERIFY_SPIN 时才允许被选出来(设计稿§2.1:边缘的不直接追)*/
        boolean spinAllowed = nowMs - lastSpinMs >= SPIN_CD_MS && remainS > SPIN_LOCKOUT_S;
        boolean justAte = nowMs < eatenHoldUntilMs;
        if (state != State.PARK && state != State.INTAKE && !justAte)
            picked = IntakeSelector.select(tracker, yawDeg, pollenBoost(remainS), nowMs, holdId,
                    spinAllowed || state == State.VERIFY_SPIN);
        BallTracker.Track pick = picked;

        long inStateMs = nowMs - stateStartMs;
        switch (state) {
            case SEARCH: {
                if (pick != null) {
                    holdId = pick.id;
                    if (Math.abs(IntakeSelector.headingErrDeg(yawDeg, pick)) > IntakeSelector.EDGE_DEG)
                        enter(State.VERIFY_SPIN, "目标在画面边缘,先转过去看清", nowMs);
                    else
                        enter(State.APPROACH, "发现目标", nowMs);
                } else if (inStateMs >= SEARCH_DWELL_MS) {//吃完的等待不会比这更久(见 eatenHoldMs),所以这里不用再判
                    turn = SEARCH_SPIN_POWER;//先原地看完 SEARCH_DWELL_MS,再转着找
                }
                break;
            }

            case APPROACH: {
                if (pick == null) {
                    /*目标出局了,分两种:一种是"看不见了",一种是"身份存疑/被拉黑/被挡路"。
                     * 只有前一种、而且球是在很近的地方看不见的,才允许盲吸——后一种绝不能开吸。
                     * identityOk 那一条是第二道保险。第一道是:holdId 只会指向候选球,而且一进SEARCH就清空。
                     * 所以按现在的写法,能走到这里时它必然成立;留着是怕以后有人改了状态的流转*/
                    BallTracker.Track held = tracker.find(holdId);
                    double heldErr = 0, heldReach = 0;
                    if (held != null) {
                        double camErr = IntakeSelector.headingErrDeg(yawDeg, held);
                        heldErr = IntakeSelector.aimErrDeg(camErr, held.distCm);//和下面一样:按吸入口中心算,不按相机
                        heldReach = IntakeSelector.reachCm(camErr, held.distCm);
                    }
                    boolean lostNearby = BLIND_INTAKE && held != null
                            && !BallTracker.isLive(held, nowMs)
                            && IntakeSelector.identityOk(tracker, held, nowMs)
                            && heldReach <= BLIND_RANGE_CM
                            && Math.abs(heldErr) <= INTAKE_TOL_DEG;
                    if (lostNearby) creepMs = creepTimeFor(heldReach);
                    enter(lostNearby ? State.INTAKE : State.SEARCH,
                            lostNearby ? "球太近看不见了,盲吸" : "目标丢失", nowMs);
                    break;
                }
                holdId = pick.id;
                double camErr = IntakeSelector.headingErrDeg(yawDeg, pick);//相机看到的:球偏右多少度
                if (Math.abs(camErr) > IntakeSelector.EDGE_DEG) {         //画面边缘是相机的事,所以这里用相机看到的角
                    enter(State.VERIFY_SPIN, "追着追着偏出去了,转回去", nowMs);//冷却中不会走到这:边缘的球此时根本选不出来
                    break;
                }
                double err   = IntakeSelector.aimErrDeg(camErr, pick.distCm);//为了让球进吸入口(而不是进相机的正前方),车头还要往右转多少度
                double reach = IntakeSelector.reachCm(camErr, pick.distCm);  //球离吸入口中心多远
                if (pick.seenThisFrame && reach <= INTAKE_RANGE_CM && Math.abs(err) <= INTAKE_TOL_DEG) {
                    creepMs = creepTimeFor(reach);
                    enter(State.INTAKE, "到位,开吸", nowMs);//只认这一帧亲眼看到的:开吸不看记忆
                    break;
                }
                double[] v = IntakeSelector.approachDrive(err, reach);
                fwd = v[0]; strafe = v[1]; turn = v[2];
                break;
            }

            case VERIFY_SPIN: {
                if (pick != null) holdId = pick.id;//别的球明显更好时跟着换(要不要换由select的滞回把关)
                BallTracker.Track held = tracker.find(holdId);
                if (held == null || !IntakeSelector.isCandidate(tracker, held, yawDeg, nowMs, true)) {
                    enter(State.SEARCH, "转过去看清了,目标不合格", nowMs);
                    break;
                }
                double err = IntakeSelector.headingErrDeg(yawDeg, held);
                if (Math.abs(err) <= SPIN_TOL_DEG && held.seenThisFrame) {
                    enter(State.APPROACH, "看清了,追", nowMs);//转到了画面中心、而且确实还看得见,才算验证通过
                    break;
                }
                if (inStateMs >= SPIN_MAX_MS) {
                    enter(State.SEARCH, "转了半天也没对准,放弃", nowMs);
                    break;
                }
                turn = IntakeSelector.MAX_TURN * Math.signum(err);//朝目标的方向原地转
                break;
            }

            case INTAKE: {
                if (arrived && enemyAtMouth) {
                    enter(State.EJECT, "吸到敌球,吐出", nowMs);//防线3
                } else if (arrived) {
                    lastBallMs = nowMs;
                    eatenHoldUntilMs = nowMs + eatenHoldMs();
                    enter(State.SEARCH, "吃到一颗", nowMs);
                } else if (inStateMs >= creepMs) {
                    tracker.ban(yawDeg, nowMs);//没吸到:这个方向先缓缓,去找别的球,免得在同一颗球上反复撞
                    enter(State.BACKOFF, "没吸到,退开", nowMs);
                } else {
                    intakePwr = INTAKE_POWER;
                    fwd = CREEP_POWER;
                }
                break;
            }

            case BACKOFF: {
                if (inStateMs >= BACKOFF_MS) enter(State.SEARCH, "退开了,重新找", nowMs);
                else fwd = -BACKOFF_POWER;
                break;
            }

            case EJECT: {
                if (inStateMs < EJECT_MS) {
                    intakePwr = -INTAKE_POWER;//先原地吐出来
                } else if (inStateMs < EJECT_MS + BACKOFF_MS) {
                    fwd = -BACKOFF_POWER;//再退开,别碾上去
                } else {
                    tracker.markEnemy(holdId);//视觉说它像友军、吸进口才发现是敌球:记仇
                    tracker.ban(yawDeg, nowMs);//这个方位拉黑;同一方位第二次吐球,本场不再来
                    enter(State.SEARCH, "已吐出并拉黑", nowMs);
                }
                break;
            }

            case PARK:
                break;//全零指令:停住,吸球电机关掉
        }
    }

    /**换状态。所有状态切换都走这里:计时归零、记下原因;需要进入动作的状态在这里处理*/
    private void enter(State next, String why, long nowMs) {
        state = next;
        stateStartMs = nowMs;
        note = why;
        if (next == State.VERIFY_SPIN) lastSpinMs = nowMs;//冷却从进入这一刻开始算
        if (next == State.SEARCH) holdId = -1;            //重新找,不念旧
    }
}
