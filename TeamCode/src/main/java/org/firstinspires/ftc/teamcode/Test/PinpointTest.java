package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Locale;

/**
 * 独立 PP 重标定工具：直接使用 FTC SDK 11.1 的官方驱动，不使用本地 Localization/Pedro。
 * 官方依据：https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf
 * 第 2-5 页：方向、分辨率、偏移、静止校零、旋转与回原点复核。
 * 180° 偏移反算是几何推导辅助，不能代替对选定参考点的机械测量。
 * 摇杆驾驶测量：只初始化 pp 和四个底盘电机，不初始化射击机构或舵机。
 */
@TeleOp(name = "Pinpoint Test", group = "Test")
public class PinpointTest extends OpMode {
    private enum Step {
        OVERVIEW, FORWARD_SIGN, LEFT_SIGN, FORWARD_SCALE, LEFT_SCALE,
        YAW_SCALE, POD_OFFSETS, OFFSET_CCW, OFFSET_CW, SPIN_VERIFY, STILL_DRIFT, RETURN_VERIFY
    }

    private static final String[] TITLES = {
            "设备/轮型", "1 前进方向", "2 左移方向", "3 前进距离比例", "4 横移距离比例",
            "5 航向比例（出厂已标定，通常只复核）", "6 吊舱到参考中心偏移：手调",
            "7 吊舱中心偏移：逆时针 180° 测算", "8 吊舱中心偏移：顺时针 180° 测算",
            "9 应用偏移后原地转动", "10 静止漂移", "11 绕行回原点"
    };
    private static final Pose2D ZERO = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    private static final double FOUR_BAR_TICKS_MM = 2000.0 / (Math.PI * 32.0);
    private static final double SWINGARM_TICKS_MM = 2000.0 / (Math.PI * 48.0);
    private static final File RESULT_FILE = new File(AppUtil.FIRST_FOLDER, "pinpoint_calibration.txt");

    private GoBildaPinpointDriver pp;
    private final DcMotor[] motors = new DcMotor[4]; // lf、rf、lb、rb
    private boolean requireNeutral = true;
    private int offsetAxis;
    private double driveScale;
    private int index, podType, firmware, deviceId, badReads;
    private boolean forwardReversed, leftReversed, forwardConfirmed, leftConfirmed;
    private double ticksPerMm = FOUR_BAR_TICKS_MM;
    private boolean customResolution, yawApplied;
    private double forwardPodYmm, strafePodXmm, yawScalar, initialYawScalar;
    private double actualDistanceMm = 1000.0;
    private int actualTurns = 10;

    private boolean measuring, calibrating;
    private long calibrationNanos, startedNanos, lastSampleNanos;
    private double previousHeading, turnedDeg, maxTurnDeg, maxRadiusMm;
    private double trialDistanceMm, trialTicksPerMm, trialYawScalar;
    private int trialTurns;
    private double minimumHeading, maximumHeading;
    private boolean trialForwardReversed, trialLeftReversed;
    private Boolean forwardCandidate, leftCandidate;
    private double forwardResolution = Double.NaN, leftResolution = Double.NaN;
    private double yawCandidate = Double.NaN;
    private final double[] ccwOffsets = {Double.NaN, Double.NaN};
    private final double[] cwOffsets = {Double.NaN, Double.NaN};
    private final String[] results = new String[Step.values().length];
    private String message = "保持静止，等待 READY；左右键选项目";
    private String saved = "尚未保存";
    private final StringBuilder history = new StringBuilder();

    @Override
    public void init() {
        Arrays.fill(results, "未测");
        String[] motorNames = {"lf", "rf", "lb", "rb"};
        for (int i = 0; i < motors.length; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, motorNames[i]);
            motors[i].setPower(0);
            motors[i].setDirection(i == 0 || i == 3 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        firmware = pp.getDeviceVersion();
        deviceId = pp.getDeviceID();
        initialYawScalar = yawScalar = pp.getYawScalar();
        // 从明确基线重新测：两个方向 FORWARD、偏移 0、默认 4-Bar。
        // Yaw 保留设备读回值，不把 1.0 冒充该设备的出厂校准值。
        applyProfile(false);
        resetAndCalibrate();
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void init_loop() {
        stopDrive();
        sample();
        display();
    }

    @Override
    public void loop() {
        sample();
        if (!measuring) {
            if (gamepad1.dpadLeftWasPressed()) index = (index + results.length - 1) % results.length;
            if (gamepad1.dpadRightWasPressed()) index = (index + 1) % results.length;
            int change = (gamepad1.dpadUpWasPressed() ? 1 : 0) - (gamepad1.dpadDownWasPressed() ? 1 : 0);
            if (step() == Step.YAW_SCALE) actualTurns = Math.max(1, Math.min(30, actualTurns + change));
            else if (step() == Step.POD_OFFSETS) {
                if (gamepad1.leftBumperWasPressed()) offsetAxis = 0;
                if (gamepad1.rightBumperWasPressed()) offsetAxis = 1;
                if (change != 0 && !calibrating && pp.getDeviceStatus() == DeviceStatus.READY) {
                    stopDrive();
                    requireNeutral = true;
                    double delta = change * (gamepad1.left_trigger > 0.2 ? 0.1 : 1.0);
                    if (offsetAxis == 0) forwardPodYmm += delta;
                    else strafePodXmm += delta;
                    applyProfile(false);
                    results[index] = format("人工设置：前进轮左右偏距 %+.3f mm，横向轮前后偏距 %+.3f mm；尚需旋转复核", forwardPodYmm, strafePodXmm);
                    message = "偏移已临时应用；松开摇杆，A 重新清零后右摇杆旋转，看过程位置残差";
                }
            } else actualDistanceMm = Math.max(100, Math.min(5000, actualDistanceMm + change * 100.0));
            if (step() == Step.OVERVIEW) {
                int podChange = (gamepad1.rightBumperWasPressed() ? 1 : 0)
                        - (gamepad1.leftBumperWasPressed() ? 1 : 0);
                if (podChange != 0) {
                    podType = (podType + 2 + podChange) % 2;
                    ticksPerMm = podType == 0 ? FOUR_BAR_TICKS_MM : SWINGARM_TICKS_MM;
                    customResolution = false;
                    forwardConfirmed = leftConfirmed = false;
                    clearDerived();
                    applyProfile(false);
                    message = "轮型已切换；请重新检查方向和距离";
                }
            }
        }
        if (gamepad1.yWasPressed()) {
            stopDrive();
            requireNeutral = true;
            if (!stationary()) message = "先松开摇杆、等车完全停稳，再按 Y 校准";
            else {
                cancelMeasurement();
                pp.recalibrateIMU();
                calibrating = true;
                calibrationNanos = System.nanoTime();
                message = "静止校准 IMU，保留当前位置；不要移动";
            }
        } else if (gamepad1.aWasPressed()) {
            beginMeasurement();
        } else if (gamepad1.bWasPressed() && measuring) {
            finishMeasurement();
        } else if (gamepad1.xWasPressed() && !measuring) {
            applyCandidate();
        }
        if (gamepad1.optionsWasPressed()) save();
        updateDrive();
        display();
    }

    private boolean sticksNeutral() {
        return Math.abs(gamepad1.left_stick_x) < 0.05 && Math.abs(gamepad1.left_stick_y) < 0.05
                && Math.abs(gamepad1.right_stick_x) < 0.05;
    }

    private boolean stationary() {
        return sticksNeutral() && Math.hypot(pp.getVelX(DistanceUnit.MM), pp.getVelY(DistanceUnit.MM)) < 5.0
                && Math.abs(pp.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)) < 1.0;
    }

    private void stopDrive() {
        driveScale = 0;
        for (DcMotor motor : motors) if (motor != null) motor.setPower(0);
    }

    private void updateDrive() {
        if (calibrating || pp.getDeviceStatus() != DeviceStatus.READY
                || (measuring && step() == Step.STILL_DRIFT)) {
            stopDrive();
            requireNeutral = true;
            return;
        }
        if (requireNeutral) {
            stopDrive();
            if (sticksNeutral()) requireNeutral = false;
            return;
        }
        // 不依赖待标定的 PP 航向：有头驾驶，前进/左移/转动均为原始底盘指令。
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        if (Math.abs(forward) < 0.05) forward = 0;
        if (Math.abs(right) < 0.05) right = 0;
        if (Math.abs(turn) < 0.05) turn = 0;
        driveScale = gamepad1.left_trigger > 0.2 ? 0.15 : 0.30;
        double power = driveScale / Math.max(1.0, Math.abs(forward) + Math.abs(right) + Math.abs(turn));
        motors[0].setPower((forward + right + turn) * power);
        motors[1].setPower((forward - right - turn) * power);
        motors[2].setPower((forward - right + turn) * power);
        motors[3].setPower((forward + right - turn) * power);
    }

    private Step step() { return Step.values()[index]; }

    private void resetAndCalibrate() {
        pp.resetPosAndIMU();
        calibrating = true;
        calibrationNanos = System.nanoTime();
    }

    private void sample() {
        pp.update();
        DeviceStatus status = pp.getDeviceStatus();
        if (status == DeviceStatus.FAULT_BAD_READ) badReads++;
        long now = System.nanoTime();
        if (calibrating && now - calibrationNanos >= 300_000_000L && status == DeviceStatus.READY) {
            calibrating = false;
        }
        if (!measuring) return;
        if (status != DeviceStatus.READY) {
            cancelMeasurement();
            message = "测量作废：设备状态 " + status + "；排查后重新 A 开始";
            return;
        }
        double heading = pp.getHeading(AngleUnit.DEGREES);
        double x = pp.getPosX(DistanceUnit.MM), y = pp.getPosY(DistanceUnit.MM);
        if (!Double.isFinite(heading) || !Double.isFinite(x) || !Double.isFinite(y)
                || now - lastSampleNanos > 500_000_000L) {
            cancelMeasurement();
            message = "测量作废：非有限读数或采样间隔超过 0.5 s，请重新测";
            return;
        }
        turnedDeg += wrap(heading - previousHeading);
        previousHeading = heading;
        lastSampleNanos = now;
        maxTurnDeg = Math.max(maxTurnDeg, Math.abs(turnedDeg));
        minimumHeading = Math.min(minimumHeading, turnedDeg);
        maximumHeading = Math.max(maximumHeading, turnedDeg);
        maxRadiusMm = Math.max(maxRadiusMm, Math.hypot(x, y));
    }

    private void applyProfile(boolean writeYaw) {
        pp.setEncoderDirections(forwardReversed ? EncoderDirection.REVERSED : EncoderDirection.FORWARD,
                leftReversed ? EncoderDirection.REVERSED : EncoderDirection.FORWARD);
        if (customResolution) pp.setEncoderResolution(ticksPerMm, DistanceUnit.MM);
        else pp.setEncoderResolution(podType == 0 ? GoBildaOdometryPods.goBILDA_4_BAR_POD
                : GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pp.setOffsets(forwardPodYmm, strafePodXmm, DistanceUnit.MM);
        if (writeYaw) pp.setYawScalar(yawScalar);
    }

    private void beginMeasurement() {
        stopDrive();
        requireNeutral = true;
        if (step() == Step.OVERVIEW) { message = "左右选择项目；本页 L1/R1 选择真实测距轮型号"; return; }
        if (calibrating || pp.getDeviceStatus() != DeviceStatus.READY) {
            message = "尚未 READY；保持静止，必要时 Y 校准"; return;
        }
        if (!stationary()) { message = "先松开摇杆、等车停稳再按 A；记录开始后用摇杆移动"; return; }
        if (step() == Step.YAW_SCALE && (!Double.isFinite(yawScalar) || yawScalar <= 0)) {
            yawScalar = pp.getYawScalar();
            if (!Double.isFinite(yawScalar) || yawScalar <= 0) {
                message = "无法读回有效 yaw scalar，不能计算或覆盖；先排查连接"; return;
            }
        }
        if (measuring) cancelMeasurement();
        // 新一轮若失败，不允许 X 误用同页上一轮留下的候选值。
        switch (step()) {
            case FORWARD_SIGN: forwardCandidate = null; break;
            case LEFT_SIGN: leftCandidate = null; break;
            case FORWARD_SCALE: forwardResolution = Double.NaN; break;
            case LEFT_SCALE: leftResolution = Double.NaN; break;
            case YAW_SCALE: yawCandidate = Double.NaN; break;
            case OFFSET_CCW: Arrays.fill(ccwOffsets, Double.NaN); break;
            case OFFSET_CW: Arrays.fill(cwOffsets, Double.NaN); break;
            default: break;
        }
        results[index] = "本轮记录中，尚无结果";
        applyProfile(false);
        if (isOffsetStep()) pp.setOffsets(0, 0, DistanceUnit.MM);
        pp.setPosition(ZERO);
        pp.update();
        if (pp.getDeviceStatus() != DeviceStatus.READY) { applyProfile(false); message = "读取未就绪，未开始"; return; }
        trialDistanceMm = actualDistanceMm;
        trialTurns = actualTurns;
        trialTicksPerMm = ticksPerMm;
        trialYawScalar = yawScalar;
        trialForwardReversed = forwardReversed;
        trialLeftReversed = leftReversed;
        previousHeading = pp.getHeading(AngleUnit.DEGREES);
        turnedDeg = maxTurnDeg = maxRadiusMm = minimumHeading = maximumHeading = 0;
        startedNanos = lastSampleNanos = System.nanoTime();
        measuring = true;
        message = "记录中；按屏幕要求移动，停稳后 B 结束；A 可重来";
    }

    private void cancelMeasurement() {
        stopDrive();
        requireNeutral = true;
        if (measuring && isOffsetStep()) applyProfile(false);
        measuring = false;
    }

    private boolean isOffsetStep() { return step() == Step.OFFSET_CCW || step() == Step.OFFSET_CW; }

    private void finishMeasurement() {
        stopDrive();
        requireNeutral = true;
        double x = pp.getPosX(DistanceUnit.MM), y = pp.getPosY(DistanceUnit.MM);
        double seconds = Math.max(0.001, (System.nanoTime() - startedNanos) / 1e9);
        String result;
        switch (step()) {
            case FORWARD_SIGN:
            case LEFT_SIGN: {
                boolean forward = step() == Step.FORWARD_SIGN;
                double primary = forward ? x : y, other = forward ? y : x;
                if (Math.abs(primary) < 100 || Math.abs(other) > Math.abs(primary) * 0.25 || maxTurnDeg > 5) {
                    result = "方向测量无效：至少直移 10 cm，勿转向；确认 X 接前进轮、Y 接横向轮";
                    break;
                }
                boolean reverse = primary < 0 ? !(forward ? trialForwardReversed : trialLeftReversed)
                        : (forward ? trialForwardReversed : trialLeftReversed);
                if (forward) forwardCandidate = reverse; else leftCandidate = reverse;
                result = format("位移 %+.1f mm，候选方向 %s；按 X 应用/确认", primary, direction(reverse));
                break;
            }
            case FORWARD_SCALE:
            case LEFT_SCALE: {
                boolean forward = step() == Step.FORWARD_SCALE;
                double measured = forward ? x : y, sideways = forward ? y : x;
                if (!forwardConfirmed || !leftConfirmed || measured < 100 || maxTurnDeg > 3
                        || Math.abs(sideways) > measured * 0.05) {
                    result = "比例测量无效：先完成两轴方向确认，再直线移动；转角≤3°、旁偏≤5%";
                    break;
                }
                double candidate = trialTicksPerMm * measured / trialDistanceMm;
                if (forward) forwardResolution = candidate; else leftResolution = candidate;
                result = format("实测 %.1f / 尺量 %.1f mm，误差 %+.2f%%，候选 %.8f ticks/mm",
                        measured, trialDistanceMm, (measured / trialDistanceMm - 1) * 100, candidate);
                break;
            }
            case YAW_SCALE: {
                double expected = trialTurns * 360.0;
                if (turnedDeg < expected * 0.8 || turnedDeg > expected * 1.2) {
                    result = "航向测量无效：须逆时针转完设置的整圈数；核对圈数/安装方向";
                    break;
                }
                yawCandidate = trialYawScalar * expected / turnedDeg;
                result = format("累计 %.3f / 实际 %.0f°，当前 %.8f -> 候选 %.8f；出厂已标定，勿轻易覆盖",
                        turnedDeg, expected, trialYawScalar, yawCandidate);
                break;
            }
            case OFFSET_CCW:
            case OFFSET_CW: {
                double expected = step() == Step.OFFSET_CCW ? 180 : -180;
                if (!forwardConfirmed || !leftConfirmed || Math.abs(turnedDeg - expected) > 15) {
                    result = "偏移测量无效：先确认方向，围绕标记参考点准确转到反向 180°，勿平移";
                    break;
                }
                // 偏移为 0 且绕参考点准确转 180°时，X=-2*strafePodX，Y=-2*forwardPodY。
                double[] offsets = step() == Step.OFFSET_CCW ? ccwOffsets : cwOffsets;
                offsets[0] = -y / 2.0;
                offsets[1] = -x / 2.0;
                result = format("候选 forwardPodY %+.3f mm，strafePodX %+.3f mm；机械尺量复核后按 X 应用",
                        offsets[0], offsets[1]);
                break;
            }
            case POD_OFFSETS:
            case SPIN_VERIFY:
                result = format("转动 %.2f°；过程最大离点 %.2f mm，终点残差 %.2f mm；须与机械偏移共同复核",
                        turnedDeg, maxRadiusMm, Math.hypot(x, y));
                break;
            case STILL_DRIFT:
                result = format("静止 %.1f s：航向漂移 %+.4f°（%+.5f°/s），峰峰 %.4f°，位置最大漂移 %.2f mm%s",
                        seconds, turnedDeg, turnedDeg / seconds, maximumHeading - minimumHeading, maxRadiusMm,
                        seconds < 10 ? "；记录不足 10 s，请加长复测" : "");
                break;
            case RETURN_VERIFY:
                result = format("回原点：X %+.2f / Y %+.2f mm，距离残差 %.2f mm，航向残差 %+.3f°（累计 %.1f°）",
                        x, y, Math.hypot(x, y), wrap(turnedDeg), turnedDeg);
                break;
            default:
                result = "未记录";
        }
        cancelMeasurement();
        results[index] = result;
        message = result;
        history.append(TITLES[index]).append(": ").append(result).append('\n');
        save();
    }

    private void applyCandidate() {
        stopDrive();
        requireNeutral = true;
        if (calibrating || pp.getDeviceStatus() != DeviceStatus.READY) {
            message = "尚未 READY，不应用参数";
            return;
        }
        switch (step()) {
            case POD_OFFSETS:
                applyProfile(false);
                message = "人工偏移已保存；A 清零，再用右摇杆旋转复核位置残差";
                save();
                return;
            case FORWARD_SIGN:
                if (forwardCandidate == null) { message = "先 A/B 测方向"; return; }
                if (forwardReversed != forwardCandidate) clearDerived();
                forwardReversed = forwardCandidate;
                forwardConfirmed = true;
                break;
            case LEFT_SIGN:
                if (leftCandidate == null) { message = "先 A/B 测方向"; return; }
                if (leftReversed != leftCandidate) clearDerived();
                leftReversed = leftCandidate;
                leftConfirmed = true;
                break;
            case FORWARD_SCALE:
            case LEFT_SCALE:
                if (!Double.isFinite(forwardResolution) || !Double.isFinite(leftResolution)) {
                    message = "先把前进和横移两个距离项目都测完；PP 只有一个共用分辨率"; return;
                }
                double difference = Math.abs(forwardResolution - leftResolution)
                        / ((forwardResolution + leftResolution) / 2.0);
                if (difference > 0.03) {
                    message = "两轴分辨率估计相差超过 3%：先查轮型、打滑、接线和尺量，不应用平均值"; return;
                }
                double newResolution = (forwardResolution + leftResolution) / 2.0;
                if (Math.abs(newResolution - ticksPerMm) > 1e-8) clearOffsets();
                ticksPerMm = newResolution;
                customResolution = true;
                break;
            case YAW_SCALE:
                if (!Double.isFinite(yawCandidate)) { message = "先 A/B 记录整圈转动"; return; }
                double ratio = yawCandidate / yawScalar;
                if (ratio < 0.95 || ratio > 1.05) {
                    message = "航向修正超 ±5%，按官方建议先排查设备/测量，不覆盖现值"; return;
                }
                yawScalar = yawCandidate;
                yawApplied = true;
                clearOffsets();
                applyProfile(true);
                message = "已临时应用 yaw 候选值；请用反向整圈独立复测，再重测偏移";
                save();
                return;
            case OFFSET_CCW:
            case OFFSET_CW:
                if (!Double.isFinite(ccwOffsets[0]) || !Double.isFinite(cwOffsets[0])) {
                    message = "先完成顺/逆时针两个 180° 偏移项目，避免只信单次旋转"; return;
                }
                forwardPodYmm = (ccwOffsets[0] + cwOffsets[0]) / 2.0;
                strafePodXmm = (ccwOffsets[1] + cwOffsets[1]) / 2.0;
                break;
            default:
                message = "本页没有可应用参数；可用 Options 保存所有结果";
                return;
        }
        applyProfile(false);
        message = "已应用到本测试设备；正式 Constants 未改。继续后续复核";
        save();
    }

    private void clearOffsets() {
        Arrays.fill(ccwOffsets, Double.NaN);
        Arrays.fill(cwOffsets, Double.NaN);
        forwardPodYmm = strafePodXmm = 0;
        results[Step.OFFSET_CCW.ordinal()] = results[Step.OFFSET_CW.ordinal()] = "配置改变，需重测";
    }

    private void clearDerived() {
        forwardResolution = leftResolution = yawCandidate = Double.NaN;
        results[Step.FORWARD_SCALE.ordinal()] = results[Step.LEFT_SCALE.ordinal()] = "配置改变，需重测";
        clearOffsets();
    }

    private void display() {
        telemetry.addData("项目", "%d/%d %s", index + 1, results.length, TITLES[index]);
        telemetry.addLine("←/→ 选项目；A 清零开始；停稳 B 结束；X 应用候选；Y 静止校准；Options 保存");
        telemetry.addLine("左摇杆平移，右摇杆旋转；默认 30% 功率，按住 L2 为 15%；B 结束立即停底盘");
        telemetry.addData("底盘", "%.0f%% | %s", driveScale * 100, requireNeutral ? "松开摇杆后恢复" : "可驾驶");
        telemetry.addData("状态", "%s | %s | ID %d FW %d | BAD_READ %d",
                pp.getDeviceStatus(), calibrating ? "校准中，保持静止" : (measuring ? "记录中" : "待测"), deviceId, firmware, badReads);
        telemetry.addData("说明", instructions());
        telemetry.addData("提示", message);
        telemetry.addData("上次结果", results[index]);
        telemetry.addData("坐标 mm", "X %+.2f | Y %+.2f", pp.getPosX(DistanceUnit.MM), pp.getPosY(DistanceUnit.MM));
        telemetry.addData("航向 deg", "当前 %+.3f | 本次累计 %+.3f", pp.getHeading(AngleUnit.DEGREES), turnedDeg);
        telemetry.addData("原始计数", "X %d | Y %d（仅诊断，不凭原始计数猜方向）", pp.getEncoderX(), pp.getEncoderY());
        telemetry.addData("速度", "X %+.1f Y %+.1f mm/s | yaw %+.3f deg/s",
                pp.getVelX(DistanceUnit.MM), pp.getVelY(DistanceUnit.MM), pp.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        if (step() == Step.POD_OFFSETS) {
            telemetry.addData("手调偏移", "%s；↑/↓ 每次 ±%s mm；L1/R1 选择轴",
                    offsetAxis == 0 ? "前进吊舱左右偏距 forwardPodY" : "横向吊舱前后偏距 strafePodX",
                    gamepad1.left_trigger > 0.2 ? "0.1" : "1.0");
            telemetry.addData("到参考中心偏差", "前进轮：%+.3f mm（左正右负）| 横向轮：%+.3f mm（前正后负）", forwardPodYmm, strafePodXmm);
        } else telemetry.addData("测量长度/圈数", "%.0f cm / CCW %d 圈；↑/↓ 改长度 10cm 或圈数 1", actualDistanceMm / 10.0, actualTurns);
        telemetry.addData("轮型", "%s | %.8f ticks/mm（两轴共用）", customResolution ? "自定义/候选" : (podType == 0 ? "4-Bar" : "Swingarm"), ticksPerMm);
        telemetry.addData("方向", "前进 %s [%s] | 横向 %s [%s]", direction(forwardReversed), forwardConfirmed ? "已确认" : "未确认",
                direction(leftReversed), leftConfirmed ? "已确认" : "未确认");
        telemetry.addData("已发偏移 mm", "forwardPodY %+.3f | strafePodX %+.3f%s", forwardPodYmm, strafePodXmm,
                measuring && isOffsetStep() ? "；测量期间临时置 0" : "");
        telemetry.addData("Yaw scalar", "初始读回 %.8f | 当前 %.8f%s", initialYawScalar, yawScalar, yawApplied ? "（本次已覆盖）" : "（未覆盖设备值）");
        if (Double.isFinite(ccwOffsets[0]) && Double.isFinite(cwOffsets[0])) {
            telemetry.addData("偏移双向差 mm", "forwardY %.3f | strafeX %.3f，差大须机械复核",
                    Math.abs(ccwOffsets[0] - cwOffsets[0]), Math.abs(ccwOffsets[1] - cwOffsets[1]));
        }
        if (measuring) telemetry.addData("记录", "%.1f s | 最大转角 %.2f° | 最大离点 %.1f mm",
                (System.nanoTime() - startedNanos) / 1e9, maxTurnDeg, maxRadiusMm);
        telemetry.addData("结果文件", saved);
        telemetry.update();
    }

    private String instructions() {
        switch (step()) {
            case OVERVIEW: return "L1/R1 选真实轮型；标签面朝上、平装。前进轮接 X、横向轮接 Y；左摇杆平移、右摇杆旋转";
            case FORWARD_SIGN: return "A 后保持朝向，左摇杆向前驾驶至少 10cm，停稳 B；应 X 增加，再 X 确认";
            case LEFT_SIGN: return "A 后保持朝向，左摇杆向左驾驶至少 10cm，停稳 B；应 Y 增加，再 X 确认";
            case FORWARD_SCALE: return "尺量标记参考点，A 后左摇杆向前驾驶设置长度，B；建议至少 100cm，避免转向";
            case LEFT_SCALE: return "尺量标记参考点，A 后左摇杆向左驾驶设置长度，B；与前进轮共用同一分辨率";
            case YAW_SCALE: return "出厂已标定通常不改。A 后慢转逆时针设置整圈数，精确对齐初始朝向再 B；↑/↓ 改圈数";
            case POD_OFFSETS: return "L1 前进轮左右偏距，R1 横向轮前后偏距；↑/↓ 调 1mm，按住 L2 调 0.1mm；A 后右摇杆转动看残差，B 后可继续改值";
            case OFFSET_CCW: return "标记车体中心参考点，A 后右摇杆逆时针准确 180°，停稳 B；参考点不可平移，测量时偏移暂为 0";
            case OFFSET_CW: return "同一中心参考点，A 后右摇杆顺时针准确 180°，停稳 B；量尺复核：前进轮左正/右负，横轮前正/后负";
            case SPIN_VERIFY: return "先应用偏移；A 后围绕参考点转整圈并反转，B；关注过程最大离点，不能只看最终回零";
            case STILL_DRIFT: return "机器人完全静止至少 10~20s；A 开始 B 结束；若持续漂移，先 Y 静止校准后重测";
            case RETURN_VERIFY: return "A 后绕行一圈，返回相同参考点和相同朝向，B；官方设置指引给出约 10mm 回点残差参考";
            default: return "";
        }
    }

    private void save() {
        try {
            File folder = RESULT_FILE.getParentFile();
            if (!folder.isDirectory() && !folder.mkdirs()) throw new java.io.IOException("无法创建 FIRST 目录");
            try (PrintWriter out = new PrintWriter(new OutputStreamWriter(new FileOutputStream(RESULT_FILE), "UTF-8"))) {
                out.println("Pinpoint Test - 候选参数/人工测量结果，非自动认证通过");
                out.println("Official: https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf");
                out.println(format("ID=%d FW=%d BAD_READ=%d; initial device yaw=%.9f", deviceId, firmware, badReads, initialYawScalar));
                out.println("仅测试设备临时配置；正式 Constants 未修改。下列片段配合当前 Constants 的 INCH 单位使用：");
                out.println(".forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection." + direction(forwardReversed) + ") // " + (forwardConfirmed ? "已确认" : "未确认"));
                out.println(".strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection." + direction(leftReversed) + ") // " + (leftConfirmed ? "已确认" : "未确认"));
                out.println(format(".forwardPodY(%.8f) // inch; mm=%.4f", forwardPodYmm / 25.4, forwardPodYmm));
                out.println(format(".strafePodX(%.8f) // inch; mm=%.4f", strafePodXmm / 25.4, strafePodXmm));
                if (customResolution) out.println(format(".customEncoderResolution(%.8f) // ticks/inch; ticks/mm=%.8f", ticksPerMm * 25.4, ticksPerMm));
                else out.println(".encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods." + (podType == 0 ? "goBILDA_4_BAR_POD" : "goBILDA_SWINGARM_POD") + ")");
                if (yawApplied) out.println(format(".yawScalar(%.9f) // 本次人工选择覆盖，须独立复测", yawScalar));
                else out.println("// yawScalar 未覆盖；使用当前设备值，不将 1.0 当作出厂标定");
                out.println("\n当前各项结果：");
                for (int i = 1; i < results.length; i++) out.println(TITLES[i] + ": " + results[i]);
                out.println("\n所有测量记录：\n" + history);
                if (out.checkError()) throw new java.io.IOException("写入失败");
            }
            saved = RESULT_FILE.getAbsolutePath();
        } catch (Exception error) { saved = "保存失败：" + error.getMessage(); }
    }

    @Override
    public void stop() { stopDrive(); cancelMeasurement(); save(); }

    private static String direction(boolean reversed) { return reversed ? "REVERSED" : "FORWARD"; }
    private static double wrap(double degrees) { return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)), Math.cos(Math.toRadians(degrees)))); }
    private static String format(String format, Object... values) { return String.format(Locale.US, format, values); }
}
