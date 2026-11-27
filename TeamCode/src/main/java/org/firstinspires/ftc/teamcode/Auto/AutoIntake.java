package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;

import java.util.Locale;

/**自动拾球(设计稿 intake-weight §5.3):让机器人自己找球、开过去、吸进来的那个自动程序。
 * 它不是吸球机构的封装——吸球机构是队友的 Intake 类,这里只是在调用它。
 * 算法本体是 IntakeFsm / IntakeSelector / BallTracker,它们不碰硬件;这个文件只做硬件读写。
 *
 * 数据流:
 *   Limelight.pollFrame     相机出新帧了吗
 *   → BallTracker.observe     把检测框串成"同一颗球的历史"
 *   → IntakeFsm.step          算法:里面调 IntakeSelector 选球,再按状态机决定怎么动
 *   → 底盘:把三个功率写给电机
 *   → 吸球机构:交给队友的 Intake 类(intake() 吸 / kick() 吐 / stop() 停)
 *
 * 视觉接的是 pipeline 3(自训练模型,标签 red/yellow/blue)。
 * 要调行为去 IntakeFsm / IntakeSelector / BallTracker 的参数墙;要改算法先在电脑上跑 Test/OfflineTest。
 *
 * 上真机之前必须做的三件事,一件都不能省:
 *  1. 先跑 Test/BallKCalib:核对标签和两个符号,再标定 K_BIG / K_POLLEN——没标定之前距离是假的,吸球时机就不可信
 *  2. 文件末尾两处 TODO(仓位检测、吸入口颜色复核)是防线3,没接通之前不要拿去打比赛
 *  3. 第一次跑把轮子架空,或者把场地清空:这个程序会让机器人自己跑*/
@Autonomous(name = "Auto Intake", group = "Auto")
public class AutoIntake extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private Intake intake;//队友的吸球机构封装:管 INTAKE 和 TRANS 两个电机。这里不再自己拿电机
    private Limelight vision;
    private BallTracker tracker;//每次开跑都新建,不带上一次的航迹和拉黑
    private IntakeFsm fsm;

    private final ElapsedTime runtime = new ElapsedTime();//开跑多久了,所有计时都以它为准

    @Override
    public void runOpMode() {
        leftFront  = driveMotor("lf", DcMotorSimple.Direction.REVERSE);//方向与 ManualOpMode 一致
        leftBack   = driveMotor("lb", DcMotorSimple.Direction.FORWARD);
        rightFront = driveMotor("rf", DcMotorSimple.Direction.FORWARD);
        rightBack  = driveMotor("rb", DcMotorSimple.Direction.REVERSE);

        intake = new Intake();
        intake.init(hardwareMap);//和 ManualOpMode 里的用法一样。config 里没有 INTAKE / TRANS 的话,这一行会报错停下

        vision = new Limelight(this);//构造函数会先切到 AprilTag 管线
        boolean switched = vision.usePipeline(Limelight.PIPE_BALLS);
        vision.setAllianceRed(false);//TODO: 临场按联盟设,true = 红方
        vision.start();

        /*相机没就绪就原地不动,把原因摆在上屏,不要带着问题开跑*/
        if (!switched) {
            telemetry.addData("无法启动", "切不到 %d 号管线:查相机供电/网线", Limelight.PIPE_BALLS);
            telemetry.update();
            while (!isStarted() && !isStopRequested()) idle();
            intake.stop();
            vision.stop();
            return;
        }

        telemetry.addData("状态", "已初始化,等待开始");
        telemetry.update();
        waitForStart();

        tracker = new BallTracker();
        runtime.reset();
        fsm = new IntakeFsm(0);

        /*try/finally:循环不管是正常结束,还是中途哪一行抛了异常,都一定会走到停车那几行。
         * 一台会自己跑的机器人,不能因为某行代码出了错就一直冲出去*/
        try {
            while (opModeIsActive()) {
                long nowMs = (long) runtime.milliseconds();//SDK里milliseconds()返回double,这里按整数毫秒用
                double yawDeg = vision.getYawDeg();//读IMU走I²C不便宜:每个循环只读这一次,视觉和决策共用

                /*视觉:出了新帧才喂给跟踪器*/
                if (vision.pollFrame(yawDeg)) tracker.observe(vision.getBallTargets(), nowMs);

                /*决策*/
                boolean secured = intakeSecured();
                fsm.step(tracker, yawDeg, IntakeFsm.PERIOD_S - runtime.seconds(),
                        vision.isConnected(), nowMs, secured, secured && intakeColorIsEnemy());

                /*执行。底盘:驱动公式和 ManualOpMode 完全一样。fwd 前进、strafe 右移、turn 右转,都是右/前为正*/
                driveMec(fsm.fwd, fsm.strafe, fsm.turn);
                setIntake(fsm.intakePwr);

                BallTracker.Track shown = (fsm.picked() != null) ? fsm.picked()
                        : (fsm.state() == IntakeFsm.State.INTAKE ? tracker.find(fsm.holdId()) : null);
                telemetry.addData("状态", "%s %dms ← %s", fsm.state(), fsm.inStateMs(nowMs), fsm.note());
                telemetry.addData("选中", shown == null ? "无" : describe(shown, yawDeg));
                telemetry.addData("航迹", "%d 条 / 拉黑 %d 处", tracker.tracks().size(), tracker.bans().size());
                telemetry.addData("相机", vision.isConnected()
                        ? String.format(Locale.ROOT, "连通 %.0f帧/秒", vision.getFps()) : "断开!");
                telemetry.addData("指令", "前%.2f 横%.2f 转%.2f 吸%.2f", fsm.fwd, fsm.strafe, fsm.turn, fsm.intakePwr);
                telemetry.update();
            }
        } finally {
            driveMec(0, 0, 0);
            intake.stop();
            vision.stop();
        }
    }

    /**把算法给的吸球功率翻译成队友 Intake 类的三个动作。
     * 队友的类没有"设功率"这个方法,功率大小由它定,所以这里只看正负号:正 = 吸,负 = 吐,零 = 停*/
    private void setIntake(double pwr) {
        if (pwr > 0) intake.intake();
        else if (pwr < 0) intake.kick();
        else intake.stop();
    }

    private void driveMec(double fwd, double strafe, double turn) {
        double d = Math.max(Math.abs(fwd) + Math.abs(strafe) + Math.abs(turn), 1.0);
        leftFront.setPower((fwd + strafe + turn) / d);
        rightFront.setPower((fwd - strafe - turn) / d);
        leftBack.setPower((fwd - strafe + turn) / d);
        rightBack.setPower((fwd + strafe - turn) / d);
    }

    private DcMotor driveMotor(String name, DcMotorSimple.Direction dir) {
        DcMotor m = hardwareMap.get(DcMotor.class, name);
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//不下指令也要停得住,不能溜车
        m.setPower(0);
        return m;
    }

    /**一行里同时显示"相机看到的"和"换算到吸入口中心之后的"。填完 CAM_OFFSET 之后可以这样验符号:
     * 把球正对相机放着,"车头转"应该指向吸入口所在的那一侧*/
    private String describe(BallTracker.Track t, double yawDeg) {
        double camErr = IntakeSelector.headingErrDeg(yawDeg, t);
        return String.format(Locale.ROOT, "%s#%d%s 相机偏%.0f° 车头转%.0f° 离吸口%.0fcm 置信%.2f", t.kind, t.id,
                t.seenThisFrame ? "" : "(凭记忆)", camErr, IntakeSelector.aimErrDeg(camErr, t.distCm),
                IntakeSelector.reachCm(camErr, t.distCm), t.conf);
    }

    /*---------- 剩余硬件缝:防线3,接上真机时填 ----------*/
    /**TODO: 仓位检测。有球就返回true,没有就返回false——如实上报就行。
     * 状态机认的是"从无到有的那一瞬间"(见 IntakeFsm.step 的说明),所以电平型、脉冲型传感器都可以用*/
    private boolean intakeSecured() { return false; }
    /**TODO: 吸入口颜色复核,刚吸到的这颗是敌球返回true(颜色传感器,或"吸到一半查Limelight末帧"的降级版)*/
    private boolean intakeColorIsEnemy() { return false; }
}
