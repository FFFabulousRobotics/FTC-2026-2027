package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PinpointLocalization;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.Locale;

/**
 * 去年方法：小功率确认方向，继电振荡辨识 P、D；I=0，F 保留当前值。
 * 再由 Pedro 主 PIDF 双向复测，稳定窗口可延长到每动作 6 秒。
 * 已测参数先保存为未验收结果，两轮达标才应用到正式 Constants 的内存对象。
 */
@Configurable
public abstract class PedroPIDFAutoTuner extends OpMode {
    public static double MAX_POWER = 0.35;
    public static double HEADING_RELAY_POWER = 0.20;
    public static double TRANSLATION_RELAY_POWER = 0.18;
    public static double HEADING_BAND_DEG = 1;
    public static double POSITION_BAND_IN = 0.35;
    public static int SKIP_CYCLES = 2;
    public static int MEASURE_CYCLES = 4;
    public static double RELAY_TIMEOUT_S = 30;
    public static double TRANSLATION_STEP_IN = 3;
    public static double HEADING_STEP_DEG = 10;
    public static double LEG_TIME_S = 6;
    public static double TAIL_TIME_S = 0.6;
    public static double POSITION_TOLERANCE_IN = 0.5;
    public static double HEADING_TOLERANCE_DEG = 1;
    public static double STOP_SPEED_IN_S = 1;
    public static double STOP_SPEED_DEG_S = 3;

    private enum State { IDLE, SIGN_OUT, SIGN_BACK, COAST, RELAY, TRIAL, MEASURED, DONE, STOPPED, FAILED }
    private State state = State.IDLE;
    private Follower follower;
    private DcMotor[] motors;
    private FollowerConstants testConstants;
    private PinpointLocalization localization = new PinpointLocalization();
    private RelayPIDFMeasurement relay;
    private double[] candidate;
    private Pose origin, target;
    private double phaseStart, lastUpdate, lastMotionTime, motionX, motionY, motionHeading;
    private double axisSign, probePower, step, legTime, tailTime, tolerance, stopSpeed;
    private double stillSince = -1, worstTailError, worstTailSpeed;
    private int leg, verified;
    private final ArrayDeque<double[]> tail = new ArrayDeque<>();
    private double tailErrorSq, tailSpeedSq, tailDuration;
    private String status = "等待开始", savedPath = "";

    protected abstract boolean isHeading();
    protected double now() { return System.nanoTime() / 1e9; }

    @Override public void init() {
        testConstants = new FollowerConstants()
                .headingPIDFCoefficients(copy(Constants.followerConstants.coefficientsHeadingPIDF))
                .translationalPIDFCoefficients(copy(Constants.followerConstants.coefficientsTranslationalPIDF))
                .translationalIntegral(0);
        testConstants.usePredictiveBraking = false;
        follower = Constants.createFollower(hardwareMap, testConstants);
        motors = new DcMotor[]{
                hardwareMap.get(DcMotor.class, Constants.mecanumConstants.leftFrontMotorName),
                hardwareMap.get(DcMotor.class, Constants.mecanumConstants.leftRearMotorName),
                hardwareMap.get(DcMotor.class, Constants.mecanumConstants.rightFrontMotorName),
                hardwareMap.get(DcMotor.class, Constants.mecanumConstants.rightRearMotorName)};
        follower.breakFollowing();
        follower.deactivateAllPIDFs();
        follower.activateHeading();
        if (!isHeading()) follower.activateTranslational();
        localization.initAuto(hardwareMap, new Pose(0, 0, 0));
        showTelemetry();
    }

    @Override public void init_loop() {
        localization.update();
        gamepad1.aWasPressed(); gamepad1.bWasPressed(); gamepad1.xWasPressed();
        showTelemetry();
    }

    @Override public void start() {
        if (localization.isReady()) follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override public void loop() {
        boolean a = gamepad1.aWasPressed(), b = gamepad1.bWasPressed(), x = gamepad1.xWasPressed();
        if (b) {
            halt(State.STOPPED, "B 已停止；A 重新辨识，X 复测已测得参数");
            showTelemetry();
            return;
        }
        double time = now();
        if (!running()) {
            if (!localization.isReady()) localization.update();
            else {
                follower.updatePose();
                if (a) begin(false, time);
                else if (x && candidate != null) begin(true, time);
            }
        } else if (time - lastUpdate > 0.25) {
            halt(State.FAILED, "循环间隔超过 0.25 s，已停机；已测参数仍保留");
        } else {
            follower.update();
            if (checkMotion(time)) {
                if (state == State.TRIAL) updateTrial(time);
                else updateMeasurement(time);
            }
        }
        lastUpdate = time;
        showTelemetry();
    }

    private void begin(boolean verifyOnly, double time) {
        double power = isHeading() ? HEADING_RELAY_POWER : TRANSLATION_RELAY_POWER;
        double band = isHeading() ? Math.toRadians(HEADING_BAND_DEG) : POSITION_BAND_IN;
        if (!Double.isFinite(MAX_POWER) || MAX_POWER <= 0 || MAX_POWER > 0.5
                || !Double.isFinite(power) || power < 0.08 || power > MAX_POWER
                || !Double.isFinite(band) || band <= 0 || band > (isHeading() ? Math.toRadians(3) : 1)
                || !Double.isFinite(TRANSLATION_STEP_IN) || TRANSLATION_STEP_IN < 2 || TRANSLATION_STEP_IN > 12
                || !Double.isFinite(HEADING_STEP_DEG) || HEADING_STEP_DEG < 5 || HEADING_STEP_DEG > 30
                || !Double.isFinite(LEG_TIME_S) || LEG_TIME_S < 1 || LEG_TIME_S > 10
                || !Double.isFinite(TAIL_TIME_S) || TAIL_TIME_S < 0.3 || TAIL_TIME_S >= LEG_TIME_S
                || !Double.isFinite(RELAY_TIMEOUT_S) || RELAY_TIMEOUT_S < 5 || RELAY_TIMEOUT_S > 90
                || !positive(POSITION_TOLERANCE_IN) || !positive(HEADING_TOLERANCE_DEG)
                || !positive(STOP_SPEED_IN_S) || !positive(STOP_SPEED_DEG_S)
                || SKIP_CYCLES < 0 || SKIP_CYCLES > 5 || MEASURE_CYCLES < 3 || MEASURE_CYCLES > 10) {
            halt(State.FAILED, "设置不合法，未开始运动");
            return;
        }
        step = isHeading() ? Math.toRadians(HEADING_STEP_DEG) : TRANSLATION_STEP_IN;
        legTime = LEG_TIME_S; tailTime = TAIL_TIME_S;
        tolerance = isHeading() ? Math.toRadians(HEADING_TOLERANCE_DEG) : POSITION_TOLERANCE_IN;
        stopSpeed = isHeading() ? Math.toRadians(STOP_SPEED_DEG_S) : STOP_SPEED_IN_S;
        origin = copy(follower.getPose());
        follower.setMaxPower(MAX_POWER);
        verified = 0;
        if (verifyOnly) {
            startVerification(time);
            return;
        }
        candidate = null;
        worstTailError = worstTailSpeed = 0;
        savedPath = "";
        relay = new RelayPIDFMeasurement(power, band, SKIP_CYCLES, MEASURE_CYCLES);
        probePower = Math.min(power, 0.18);
        phaseStart = lastUpdate = time;
        resetMotion(time);
        state = State.SIGN_OUT;
        status = "先短距离确认测试轴方向，再做小幅往复辨识";
        follower.startTeleopDrive(true);
        raw(probePower);
    }

    private void updateMeasurement(double time) {
        double elapsed = time - phaseStart;
        if (state == State.SIGN_OUT && elapsed >= 0.35) {
            double moved = displacement();
            if (Math.abs(moved) < (isHeading() ? Math.toRadians(0.8) : 0.1)) {
                halt(State.FAILED, "方向探测未得到有效位移；检查底盘/测距轮，或适当增加继电功率");
                return;
            }
            axisSign = Math.signum(moved);
            state = State.SIGN_BACK; phaseStart = time;
            raw(-probePower);
        } else if (state == State.SIGN_BACK && elapsed >= 0.35) {
            raw(0);
            state = State.COAST; phaseStart = time; stillSince = -1;
        } else if (state == State.COAST) {
            if (speed() <= stopSpeed) {
                if (stillSince < 0) stillSince = time;
                if (time - stillSince >= 0.3) {
                    origin = copy(follower.getPose());
                    state = State.RELAY; phaseStart = time;
                    resetMotion(time);
                    raw(relay.update(time, 0) * axisSign);
                }
            } else stillSince = -1;
            if (state == State.COAST && elapsed > 4) halt(State.FAILED, "零功率后仍未停止，已结束探测");
        } else if (state == State.RELAY) {
            raw(relay.update(time, -displacement()) * axisSign);
            if (relay.ready()) {
                follower.breakFollowing();
                candidate = relay.gains(sharedCoefficients().F);
                saveResult(false);
                status = "P、D 已测得，I=0、F 保留当前值；开始闭环复测";
                startVerification(time);
            } else if (elapsed > RELAY_TIMEOUT_S) {
                halt(State.FAILED, "未测到足够稳定的振荡周期；检查 telemetry 幅度/功率后重试");
            }
        }
    }

    private void raw(double output) {
        if (isHeading()) follower.setTeleOpDrive(0, 0, output, true);
        else follower.setTeleOpDrive(0, output, 0, true);
    }

    private double displacement() {
        Pose p = follower.getPose();
        if (isHeading()) return wrap(p.getHeading() - origin.getHeading());
        return -(p.getX() - origin.getX()) * Math.sin(origin.getHeading())
                + (p.getY() - origin.getY()) * Math.cos(origin.getHeading());
    }

    private void startVerification(double time) {
        // 不依赖旧 PID 强行回原点；从当前位置建立复测中心。
        origin = copy(follower.getPose());
        PIDFCoefficients coefficients = isHeading() ? testConstants.coefficientsHeadingPIDF
                : testConstants.coefficientsTranslationalPIDF;
        coefficients.setCoefficients(candidate[0], candidate[1], candidate[2], candidate[3]);
        leg = 0;
        worstTailError = worstTailSpeed = 0;
        state = State.TRIAL;
        beginLeg(time);
    }

    private void beginLeg(double time) {
        double dx = 0, dy = 0, heading = origin.getHeading();
        if (isHeading()) heading += (leg == 0 ? step : leg == 2 ? -step : 0);
        else if (leg < 4) {
            int direction = verified == 0 ? leg : 3 - leg;
            dx = direction == 0 ? step : direction == 2 ? -step : 0;
            dy = direction == 1 ? step : direction == 3 ? -step : 0;
        }
        double c = Math.cos(origin.getHeading()), s = Math.sin(origin.getHeading());
        target = new Pose(origin.getX() + dx * c - dy * s,
                origin.getY() + dx * s + dy * c, wrap(heading));
        follower.holdPoint(target, false);
        phaseStart = lastUpdate = time;
        tail.clear(); tailErrorSq = tailSpeedSq = tailDuration = 0;
        resetMotion(time);
    }

    private void updateTrial(double time) {
        double dt = time - lastUpdate, e = error(), v = speed();
        tail.addLast(new double[]{lastUpdate, time, e * e, v * v});
        tailErrorSq += e * e * dt; tailSpeedSq += v * v * dt; tailDuration += dt;
        double cutoff = time - tailTime;
        while (!tail.isEmpty() && tail.peekFirst()[0] < cutoff) {
            double[] sample = tail.peekFirst();
            double removed = Math.min(sample[1], cutoff) - sample[0];
            tailErrorSq -= sample[2] * removed; tailSpeedSq -= sample[3] * removed; tailDuration -= removed;
            sample[0] += removed;
            if (sample[1] <= cutoff) tail.removeFirst(); else break;
        }
        if (tailDuration < tailTime * 0.99) return;
        double rms = Math.sqrt(Math.max(0, tailErrorSq / tailDuration));
        double rmsSpeed = Math.sqrt(Math.max(0, tailSpeedSq / tailDuration));
        boolean settled = rms <= tolerance && rmsSpeed <= stopSpeed;
        if (time - phaseStart < 0.8 || (!settled && time - phaseStart < legTime)) return;
        worstTailError = Math.max(worstTailError, rms);
        worstTailSpeed = Math.max(worstTailSpeed, rmsSpeed);
        if (++leg < (isHeading() ? 4 : 5)) {
            beginLeg(time);
            return;
        }
        follower.breakFollowing();
        if (worstTailError > tolerance || worstTailSpeed > stopSpeed) {
            state = State.MEASURED;
            status = String.format(Locale.US, "参数已测得，复测未达标：RMS %.3f/%.3f，速度 %.3f/%.3f；X 可重测",
                    displayError(worstTailError), displayError(tolerance), displayError(worstTailSpeed), displayError(stopSpeed));
            saveResult(false);
        } else if (++verified < 2) {
            startVerification(time);
        } else {
            publishResult();
            state = State.DONE;
        }
    }

    private boolean checkMotion(double time) {
        Pose p = follower.getPose();
        double drift = Math.hypot(p.getX() - origin.getX(), p.getY() - origin.getY());
        double turn = Math.abs(wrap(p.getHeading() - origin.getHeading()));
        double positionLimit = state == State.TRIAL ? step + 8 : 8;
        if (localization.getDriver().getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(drift) || !Double.isFinite(turn) || !Double.isFinite(speed())
                || drift > (isHeading() ? 4 : positionLimit)
                || turn > (isHeading() ? Math.toRadians(50) : Math.toRadians(20))) {
            halt(State.FAILED, "定位异常或位移/转角越界，已停机；已测参数仍保留");
            return false;
        }
        double changed = isHeading() ? Math.abs(wrap(p.getHeading() - motionHeading))
                : Math.hypot(p.getX() - motionX, p.getY() - motionY);
        if (changed > (isHeading() ? Math.toRadians(0.15) : 0.03)) resetMotion(time);
        double power = 0;
        for (DcMotor motor : motors) power = Math.max(power, Math.abs(motor.getPower()));
        if (power < 0.12 || (state == State.TRIAL && error() <= tolerance * 3)) lastMotionTime = time;
        if (time - lastMotionTime > 1.5) {
            halt(State.FAILED, "持续输出但定位无位移，已停机；检查测距轮/电机方向");
            return false;
        }
        return true;
    }

    private void resetMotion(double time) {
        Pose p = follower.getPose();
        motionX = p.getX(); motionY = p.getY(); motionHeading = p.getHeading(); lastMotionTime = time;
    }

    private double error() {
        Pose p = follower.getPose();
        return isHeading() ? Math.abs(wrap(target.getHeading() - p.getHeading()))
                : Math.hypot(target.getX() - p.getX(), target.getY() - p.getY());
    }

    private double speed() {
        return isHeading() ? Math.abs(follower.getAngularVelocity()) : follower.getVelocity().getMagnitude();
    }

    private PIDFCoefficients sharedCoefficients() {
        return isHeading() ? Constants.followerConstants.coefficientsHeadingPIDF
                : Constants.followerConstants.coefficientsTranslationalPIDF;
    }

    private void publishResult() {
        sharedCoefficients().setCoefficients(candidate[0], candidate[1], candidate[2], candidate[3]);
        if (isHeading()) Constants.followerConstants.useSecondaryHeadingPIDF = false;
        else {
            Constants.followerConstants.useSecondaryTranslationalPIDF = false;
            Constants.followerConstants.usePredictiveBraking = false;
            Constants.followerConstants.integralTranslational.setCoefficients(0, 0, 0, 0);
        }
        status = "两轮复测通过，已应用到当前进程；永久保存仍需写回 Constants.java";
        saveResult(true);
    }

    private void saveResult(boolean passed) {
        String result = "// " + (passed ? "VERIFIED: two closed-loop passes" : "MEASURED ONLY: not accepted/applied")
                + "\n// Heading errors: radians; translation errors: inches.\n" + coefficientLine() + "\n"
                + (isHeading() ? ".useSecondaryHeadingPIDF(false)\n"
                : ".useSecondaryTranslationalPIDF(false)\n.translationalIntegral(0.0)\n"
                + "// Keep followerConstants.usePredictiveBraking = false for PIDF control.\n")
                + String.format(Locale.US, "// Ku=%.6f Tu=%.6f amplitude=%.6f\n",
                relay.ultimateGain, relay.period, relay.amplitude)
                + (state == State.RELAY ? "// Closed-loop verification has not run yet.\n"
                : String.format(Locale.US, "// Verification RMS=%.6f speed=%.6f\n", worstTailError, worstTailSpeed));
        File file = new File(AppUtil.FIRST_FOLDER,
                isHeading() ? "pedro_heading_pidf_autotune.txt" : "pedro_translational_pidf_autotune.txt");
        try (FileWriter writer = new FileWriter(file)) {
            writer.write(result); savedPath = file.getAbsolutePath();
        } catch (IOException e) { savedPath = "保存失败：" + e.getMessage() + "；请拍下参数"; }
        RobotLog.ii("PedroPIDFAutoTuner", result);
    }

    private String coefficientLine() {
        return String.format(Locale.US, ".%sPIDFCoefficients(new PIDFCoefficients(%.8f, %.8f, %.8f, %.8f))",
                isHeading() ? "heading" : "translational", candidate[0], candidate[1], candidate[2], candidate[3]);
    }

    private boolean running() {
        return state == State.SIGN_OUT || state == State.SIGN_BACK || state == State.COAST
                || state == State.RELAY || state == State.TRIAL;
    }

    private void halt(State next, String message) { follower.breakFollowing(); state = next; status = message; }
    @Override public void stop() { if (follower != null) follower.breakFollowing(); state = State.STOPPED; }

    private void showTelemetry() {
        telemetry.addData("模式", isHeading() ? "航向：继电辨识 + 复测" : "平移：继电辨识 + 复测");
        telemetry.addData("状态", state);
        telemetry.addData("Pinpoint", localization.isReady() ? "READY" : "校准中，请保持静止");
        telemetry.addLine("START 后 A 自动辨识；B 停止；X 复测已测得参数");
        telemetry.addLine(status);
        if (state == State.TRIAL && tailDuration > 0) {
            telemetry.addData(isHeading() ? "窗口误差/阈值 (deg)" : "窗口误差/阈值 (in)", "%.3f / %.3f",
                    displayError(Math.sqrt(Math.max(0, tailErrorSq / tailDuration))), displayError(tolerance));
            telemetry.addData(isHeading() ? "窗口速度/阈值 (deg/s)" : "窗口速度/阈值 (in/s)", "%.3f / %.3f",
                    displayError(Math.sqrt(Math.max(0, tailSpeedSq / tailDuration))), displayError(stopSpeed));
        }
        if (relay != null) {
            telemetry.addData("测量周期", "%d/%d（另丢弃起步 %d 周期）", relay.measuredCycles(), MEASURE_CYCLES, SKIP_CYCLES);
            telemetry.addData("Ku / Tu (s)", "%.5f / %.3f", relay.ultimateGain, relay.period);
            telemetry.addData(isHeading() ? "振幅 (deg)" : "振幅 (in)", "%.3f", displayError(relay.amplitude));
        }
        if (candidate != null) {
            telemetry.addData("P / I / D / F", "%.6f / %.6f / %.6f / %.6f",
                    candidate[0], candidate[1], candidate[2], candidate[3]);
            telemetry.addLine("I 固定 0；F 保留原值。P、D 为本次辨识结果。");
            telemetry.addData("已通过复测", "%d/2", verified);
            telemetry.addData(isHeading() ? "本轮最差 RMS (deg)" : "本轮最差 RMS (in)", "%.3f", displayError(worstTailError));
            telemetry.addData(isHeading() ? "本轮最差速度 (deg/s)" : "本轮最差速度 (in/s)", "%.3f", displayError(worstTailSpeed));
            telemetry.addLine(coefficientLine());
        }
        if (state == State.TRIAL) {
            telemetry.addData("动作", "%d/%d", leg + 1, isHeading() ? 4 : 5);
            telemetry.addData(isHeading() ? "实时误差 (deg)" : "实时误差 (in)", "%.3f", displayError(error()));
            telemetry.addData("当前动作时间 (s)", "%.1f / %.1f", now() - phaseStart, legTime);
        }
        if (!savedPath.isEmpty()) telemetry.addData("结果文件", savedPath);
        telemetry.update();
    }

    private double displayError(double value) { return isHeading() ? Math.toDegrees(value) : value; }
    private static boolean positive(double value) { return Double.isFinite(value) && value > 0; }
    private static double wrap(double angle) { return Math.atan2(Math.sin(angle), Math.cos(angle)); }
    private static Pose copy(Pose p) { return new Pose(p.getX(), p.getY(), p.getHeading()); }
    private static PIDFCoefficients copy(PIDFCoefficients c) { return new PIDFCoefficients(c.P, c.I, c.D, c.F); }
}
