package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro 2.1.2：麦轮底盘 + Pinpoint，Tuning 和自动共用。
 * 待填项用 // 注释，括号内留空；填完再取消注释，未启用的项使用库默认值。
 * 距离 inch，速度 inch/s，加速度 inch/s²，朝向/角度误差 rad。
 * Tuning 顺序：Localization → Automatic → Manual → Tests。
 * Tuning 临时应用、Panels 修改的测量值，最后都要写回这里。
 */
@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            // 整机质量，kg；用秤称含电池的比赛状态整机，没有对应 Tuning 项。
            // .mass( )
            // Automatic > Forward Zero Power Acceleration Tuner：前进断电滑行减速度，填负值。
            // .forwardZeroPowerAcceleration( )
            // Automatic > Lateral Zero Power Acceleration Tuner：横移断电滑行减速度，填负值。
            // .lateralZeroPowerAcceleration( )

            // Translational PIDF Auto Tuner 两轮复测通过；RMS 0.453 in，速度 0.974 in/s。
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25714286, 0.00000000, 0.01966863, 0.00000000))
            .useSecondaryTranslationalPIDF(false)
            .translationalIntegral(0.0)
            // Heading PIDF Auto Tuner 实测初值；复测 RMS 1.130°，先用于后续底盘调试。
            .headingPIDFCoefficients(new PIDFCoefficients(3.38097802, 0.00000000, 0.17034056, 0.01000000))
            .useSecondaryHeadingPIDF(false)
            // Manual > Drive Tuner：沿路径的前进/减速控制；顺序 P、I、D、T、F。
            // T 是 D 项滤波参数，不是跑打提前量。
            // .drivePIDFCoefficients(new FilteredPIDFCoefficients( , , , , ))
            // Manual > Centripetal Tuner：弯道向心力补偿，纠正高速转弯时向外甩。
            // .centripetalScaling( )

            // 可选小误差 PID：仍跑对应的 Translational / Heading / Drive Tuner。
            // 填入 secondary 系数会自动启用该组；不用就保持注释。
            // .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients( , , , ))
            // .translationalPIDFSwitch( ) // 平移误差切换阈值，inch。
            // .secondaryHeadingPIDFCoefficients(new PIDFCoefficients( , , , ))
            // .headingPIDFSwitch( ) // 朝向误差切换阈值，rad。
            // .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients( , , , , ))
            // .drivePIDFSwitch( ) // Drive 误差切换阈值，inch/s。
            // .useSecondaryTranslationalPIDF( ) // 单独控制开关，boolean。
            // .useSecondaryHeadingPIDF( )
            // .useSecondaryDrivePIDF( )

            // 可选预测刹车，与传统 Drive/PID 调法分开使用；此调用会自动开启预测刹车。
            // Automatic > Predictive Braking Tuner 输出 kLinearBraking、kQuadraticFriction；
            // 再用 Tests > Line 调 P。构造顺序：P、kLinearBraking、kQuadraticFriction。
            // .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients( , , ))

            // 终点保持：无专门自动测量项，用 Tests > Line / Triangle / Circle 检查。
            // .automaticHoldEnd( ) // 到达路径终点后是否保持位置，boolean。
            // .holdPointTranslationalScaling( ) // 保持位置时的平移纠偏比例。
            // .holdPointHeadingScaling( ) // 保持位置时的朝向纠偏比例。
            // .turnHeadingErrorThreshold( ) // turn/turnTo 完成的朝向误差，rad。

            // 高级滤波：无单独测量项，在 Manual > Drive Tuner 看速度噪声后再调整。
            // .driveKalmanFilterModelCovariance( ) // Q，过程噪声权重。
            // .driveKalmanFilterDataCovariance( ) // R，测量噪声权重。
            // .BEZIER_CURVE_SEARCH_LIMIT( ) // 最近路径点搜索次数，整数。
            // 以下为库自带卡住判定，无专门 Tuning 项，通常保持默认。
            // .stuckVelocity( ) // 低速阈值，inch/s。
            // .stuckTValueLow( ) // 检测开始的路径参数 t，避开起步。
            // .stuckTValueHigh( ) // 检测结束的路径参数 t，避开终点减速。
            // .stuckTimeout( ) // 持续低速多久后按卡住结束路径，ms。
            ;

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            // 当前 Robot Config 名称和手动底盘方向；Localization > Localization Test 核对。
            .leftFrontMotorName("lf").leftRearMotorName("lb")
            .rightFrontMotorName("rf").rightRearMotorName("rb")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            // Automatic > Forward Velocity Tuner：满功率稳定前进速度，inch/s。
            // .xVelocity( )
            // Automatic > Lateral Velocity Tuner：满功率稳定横移速度，inch/s，填大小。
            // .yVelocity( )
            // 功率上限，0～1；无测量项，用 Tests > Line / Triangle 逐步验证。
            // .maxPower( )
            // 指令变化达到多少才重写电机功率；无 Tuning 项，通常保持默认。
            // .motorCachingThreshold( )
            // Pedro 自己控制 TeleOp 时的零功率刹车，不改变当前独立 ManualOpMode。
            // .useBrakeModeInTeleOp( )
            // 电压补偿：无专门 Tuning，在不同电池电压下用 Tests > Line 比较。
            // .useVoltageCompensation( )
            // .nominalVoltage( ) // 参考电压，V。
            // .staticFrictionCoefficient( ) // 电压补偿模型中的静摩擦参数。
            ;

    // 手动和测试的 PinpointLocalization 也读取此对象，避免安装参数不同步。
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pp")
            .distanceUnit(DistanceUnit.INCH)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            // 横向测距轮反装：在编码器源头修正，所有模块统一读取修正后的位姿和速度。
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            // Localization > Offsets Tuner：先将下面两个偏移填 0，再原地转约 180°。
            // 读取 forwardY / strafeX，输出单位是 inch，可直接填回。
            .forwardPodY(0.070778) // Offsets Tuner 清零后实测；左正、右负，inch。
            .strafePodX(-1.264770) // Offsets Tuner 清零后实测；前正、后负，inch。
            // 与去年 CZL 相同的 goBILDA 4-Bar 测距轮，使用内置分辨率。
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // 非标准测距轮才填自定义分辨率；当前单位 INCH，因此这里是 ticks/inch。
            // Localization > Forward Tuner / Lateral Tuner：对比实测距离，检查比例。
            // .customEncoderResolution( )
            // Localization > Turn Tuner：比较实际转角，存在稳定比例误差时才调整。
            // .yawScalar( )
            // 编码器方向修改上面两行；Localization Test 中前进/左移时对应计数应增加。
            ;

    public static PathConstraints pathConstraints = PathConstraints.defaultConstraints.copy();

    static {
        // 以下不是自动测量结果：用 Tests > Line / Triangle / Circle 观察停车后调整。
        // pathConstraints.setTValueConstraint( ); // 路径进度进入终点段的阈值，0～1。
        // pathConstraints.setVelocityConstraint( ); // 终点允许的剩余速度，inch/s。
        // pathConstraints.setTranslationalConstraint( ); // 终点允许的位置误差，inch。
        // pathConstraints.setHeadingConstraint( ); // 终点允许的朝向误差，rad。
        // pathConstraints.setTimeoutConstraint( ); // 进入终点段后的继续纠偏时限，ms。
        // Manual > Drive Tuner / Tests > Line：改变减速力度和开始减速的位置。
        // pathConstraints.setBrakingStrength( ); // 刹车强度倍率。
        // pathConstraints.setBrakingStart( ); // 开始刹车位置的倍率。
        // pathConstraints.setBEZIER_CURVE_SEARCH_LIMIT( ); // 最近路径点搜索次数，整数。

        // 高级项：无单独测量器，需要 secondary 平移积分时才填。
        // followerConstants.setIntegralSecondaryTranslational(new PIDFCoefficients( , , , ));
        // 预测刹车反向制动的功率上限，0～1；用 Tests > Line 验证。
        // followerConstants.predictiveBrakingCoefficients.withMaximumBrakingPower( );
        // 当前使用已标定的平移 PIDF，关闭会绕过它的预测刹车模式。
        followerConstants.usePredictiveBraking = false;
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap, followerConstants);
    }

    /** 调参器使用独立的控制参数，候选值不会提前覆盖正式参数。 */
    public static Follower createFollower(HardwareMap hardwareMap, FollowerConstants controlConstants) {
        // 2.1.2 的 x/yVelocity setter 不会重算轮子向量，这里根据填写的速度同步。
        mecanumConstants.frontLeftVector = new Vector(1,
                Math.atan2(-mecanumConstants.yVelocity, mecanumConstants.xVelocity));
        return new FollowerBuilder(controlConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
