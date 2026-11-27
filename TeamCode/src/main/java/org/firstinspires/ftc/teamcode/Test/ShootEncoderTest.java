package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Shoot;

import java.util.ArrayDeque;
import java.util.ArrayList;

@Configurable
@TeleOp(name = "Shoot Encoder Test", group = "Test")
public class ShootEncoderTest extends OpMode {
    public static double TARGET_VELOCITY = 1000;
    public static double P = Shoot.P, I = Shoot.I, D = Shoot.D, F = Shoot.F;
    public static double LOW_POWER = 0.30, HIGH_POWER = 0.55;
    public static double STEP_TIME_S = 4.0;
    public static double ERROR_TOLERANCE = 15;
    public static double ERROR_WINDOW_S = 2.0;
    public static double MEAN_ERROR_TOLERANCE = 5;
    public static double MIN_WITHIN_RATIO = 0.95;

    private DcMotorEx leftflywheel, rightflywheel;
    private final ElapsedTime clock = new ElapsedTime();
    private final Controller controller = new Controller();

    @Override
    public void init() {
        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        for (DcMotorEx motor : new DcMotorEx[]{leftflywheel, rightflywheel}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        showTelemetry(0);
    }

    @Override
    public void start() {
        clock.reset();
        controller.stop();
    }

    @Override
    public void loop() {
        double now = clock.seconds();
        if (gamepad1.yWasPressed()) TARGET_VELOCITY += 50;
        if (gamepad1.aWasPressed()) TARGET_VELOCITY -= 50;
        TARGET_VELOCITY = Range.clip(TARGET_VELOCITY, 0, Shoot.MAX_TARGET_VELOCITY_TICKS_S);
        if (gamepad1.xWasPressed()) {
            if (controller.state == State.STOPPED || controller.state == State.FAILED) {
                if (controller.tuned) controller.run(now);
                else controller.beginTune(now);
            } else controller.stop();
        }
        if (gamepad1.bWasPressed()) controller.beginTune(now);

        double measured = Math.abs(rightflywheel.getVelocity());
        double power = controller.update(now, measured, TARGET_VELOCITY);
        leftflywheel.setPower(power);
        rightflywheel.setPower(power);
        showTelemetry(measured);
    }

    private void showTelemetry(double measured) {
        telemetry.addLine("X 开始/停止；首次自动标定，之后恢复闭环；B 重新标定；Y/A 目标 ±50");
        telemetry.addLine("标定时不送球：两档功率各运行 STEP_TIME_S 秒，完成后自动闭环");
        telemetry.addData("state", controller.state);
        telemetry.addData("target", "%.0f", TARGET_VELOCITY);
        telemetry.addData("currentvelocity", "%.1f", measured);
        telemetry.addData("error", "%.1f", TARGET_VELOCITY - measured);
        telemetry.addData("power (both)", "%.3f", controller.power);
        telemetry.addData("P / I / D / F", "%.8f / %.8f / %.8f / %.8f", P, I, D, F);
        telemetry.addData("mean error / RMS (ticks/s)", "%.1f / %.1f",
                controller.errors.meanError, Math.sqrt(controller.errors.meanSquareError));
        telemetry.addData("within tolerance (%)", "%.1f", 100 * controller.errors.withinRatio);
        telemetry.addData("peak error (ticks/s)", "%.1f", controller.errors.peakError);
        telemetry.addData("window measured / required (s)", "%.2f / %.2f",
                controller.errors.duration, ERROR_WINDOW_S);
        telemetry.addData("stable (window)", controller.errors.stable);
        telemetry.addData("low / high speed (ticks/s)", "%.1f / %.1f", controller.lowSpeed, controller.highSpeed);
        telemetry.addData("gain / response time (s)", "%.1f / %.3f", controller.gain, controller.tau);
        if (!controller.message.isEmpty()) telemetry.addLine(controller.message);
        telemetry.update();
    }

    @Override
    public void stop() {
        controller.stop();
        leftflywheel.setPower(0);
        rightflywheel.setPower(0);
    }

    enum State { STOPPED, LOW_STEP, HIGH_STEP, RUNNING, FAILED }

    /** 按采样时长加权的滑动窗口；所有跳点仍计入 RMS、峰值和超差比例。 */
    static class ErrorWindow {
        private final ArrayDeque<double[]> samples = new ArrayDeque<>();
        private double lastTime = -1;
        double duration, meanError, meanSquareError, withinRatio, peakError;
        boolean stable;

        void clear() {
            samples.clear();
            lastTime = -1;
            duration = meanError = meanSquareError = withinRatio = peakError = 0;
            stable = false;
        }

        void add(double now, double error) {
            if (lastTime < 0) {
                lastTime = now;
                return;
            }
            // 长时间没有采样不能当作已连续观察；每次最多记入 0.1 秒。
            double start = Math.max(lastTime, now - 0.1);
            lastTime = now;
            if (now > start) samples.addLast(new double[]{start, now, error});
            double cutoff = now - ERROR_WINDOW_S;
            while (!samples.isEmpty() && samples.peekFirst()[1] <= cutoff) samples.removeFirst();
            if (!samples.isEmpty()) samples.peekFirst()[0] = Math.max(samples.peekFirst()[0], cutoff);

            duration = meanError = meanSquareError = withinRatio = peakError = 0;
            for (double[] sample : samples) {
                double weight = sample[1] - sample[0];
                double e = sample[2];
                duration += weight;
                meanError += e * weight;
                meanSquareError += e * e * weight;
                if (Math.abs(e) <= ERROR_TOLERANCE) withinRatio += weight;
                peakError = Math.max(peakError, Math.abs(e));
            }
            if (duration > 0) {
                meanError /= duration;
                meanSquareError /= duration;
                withinRatio /= duration;
            }
            stable = duration >= ERROR_WINDOW_S - 1e-6
                    && Math.abs(meanError) <= MEAN_ERROR_TOLERANCE
                    && Math.sqrt(meanSquareError) <= ERROR_TOLERANCE
                    && withinRatio >= MIN_WITHIN_RATIO - 1e-9;
        }
    }

    /** 两档阶跃辨识一阶速度模型，再用保守 lambda 调法得到 PI；D 默认不用。 */
    static class Controller {
        State state = State.STOPPED;
        boolean tuned;
        String message = "";
        double power, lowSpeed, highSpeed, gain, tau;
        final ErrorWindow errors = new ErrorWindow();
        private double phaseStart, lastTime, integralPower, previousVelocity, derivative;
        private double previousTarget = -1;
        private double speedSum, sampleCount, lowPower, highPower, stepTime;
        private final ArrayList<double[]> response = new ArrayList<>();

        void beginTune(double now) {
            stop();
            lowPower = Range.clip(LOW_POWER, 0, 1);
            highPower = Range.clip(HIGH_POWER, 0, 1);
            stepTime = STEP_TIME_S;
            if (highPower <= lowPower || stepTime < 2) {
                fail("标定设置无效：HIGH_POWER 应大于 LOW_POWER，STEP_TIME_S 至少 2 秒");
                return;
            }
            state = State.LOW_STEP;
            phaseStart = lastTime = now;
            lowSpeed = highSpeed = gain = tau = 0;
            speedSum = sampleCount = 0;
            response.clear();
        }

        void run(double now) {
            state = State.RUNNING;
            lastTime = now;
            integralPower = derivative = 0;
            errors.clear();
            previousTarget = -1;
            message = "";
        }

        void stop() {
            state = State.STOPPED;
            power = integralPower = derivative = 0;
            errors.clear();
            message = "";
        }

        double update(double now, double measured, double target) {
            double dt = Range.clip(now - lastTime, 0.001, 0.1);
            lastTime = now;
            double elapsed = now - phaseStart;
            // 每档最后 1 秒取平均，减小单次编码器测速噪声。
            if (state == State.LOW_STEP || state == State.HIGH_STEP) {
                if (elapsed >= stepTime - 1.0) {
                    speedSum += measured;
                    sampleCount++;
                }
            }
            if (state == State.LOW_STEP) {
                power = lowPower;
                if (elapsed >= stepTime) {
                    lowSpeed = speedSum / Math.max(1, sampleCount);
                    if (lowSpeed < 20) return fail("未测到有效右编码器转速，标定停止");
                    phaseStart = now;
                    speedSum = sampleCount = 0;
                    response.add(new double[]{0, lowSpeed});
                    state = State.HIGH_STEP;
                    power = highPower;
                }
                return power;
            }
            if (state == State.HIGH_STEP) {
                response.add(new double[]{elapsed, measured});
                power = highPower;
                if (elapsed < stepTime) return power;
                highSpeed = speedSum / Math.max(1, sampleCount);
                double speedChange = highSpeed - lowSpeed;
                if (speedChange < 30) return fail("两档转速差不足，检查右编码器或增大阶跃后重测");
                gain = speedChange / (highPower - lowPower);
                double threshold = lowSpeed + 0.63212056 * speedChange;
                tau = 0;
                for (int i = 1; i < response.size(); i++) {
                    double[] sample = response.get(i), before = response.get(i - 1);
                    if (sample[1] >= threshold) {
                        double fraction = (threshold - before[1]) / (sample[1] - before[1]);
                        tau = before[0] + fraction * (sample[0] - before[0]);
                        break;
                    }
                }
                if (tau <= 0) return fail("没有测到有效阶跃响应，请增大 STEP_TIME_S 后重测");
                // 忽略纯延迟的一阶近似：Kp=tau/(gain*lambda)，Ki=Kp/tau。
                double lambda = Math.max(0.5, 2 * tau);
                P = tau / (gain * lambda);
                I = P / tau;
                D = 0;
                F = 1.0 / gain;
                tuned = true;
                run(now);
            }
            if (state != State.RUNNING) return power = 0;
            target = Range.clip(target, 0, Shoot.MAX_TARGET_VELOCITY_TICKS_S);
            if (target != previousTarget) {
                previousTarget = target;
                previousVelocity = measured;
                derivative = 0;
                errors.clear();
            }
            if (target < 1) {
                integralPower = 0;
                errors.clear();
                previousVelocity = measured;
                return power = 0;
            }

            double error = target - measured;
            // D 取实测速度的负导数并滤波，避免目标跳变引发微分冲击。
            derivative += dt / (0.05 + dt) * (-(measured - previousVelocity) / dt - derivative);
            previousVelocity = measured;
            double feedAndFeedback = F * target + P * error + D * derivative;
            double candidate = Range.clip(integralPower + I * error * dt, -1, 1);
            double raw = feedAndFeedback + candidate;
            // 饱和时不继续向同一方向积累积分，目标降低后能及时恢复。
            if ((raw >= 0 && raw <= 1) || (raw > 1 && error < 0) || (raw < 0 && error > 0)) {
                integralPower = candidate;
            }
            power = Range.clip(feedAndFeedback + integralPower, 0, 1);
            errors.add(now, error);
            return power;
        }

        private double fail(String reason) {
            stop();
            state = State.FAILED;
            message = reason;
            return 0;
        }
    }
}
