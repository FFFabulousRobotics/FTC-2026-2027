package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Configurable
public class Turret {
    // 去年云台的初始参数；今年需实测传动比、舵机行程、零位、预紧和轴心偏移。
    public static double TURRET_GEAR_RATIO = 0.89;
    public static double TURRET_SERVO_RANGE_DEG = 353;
    public static double TURRET_SERVO_OFFSET = 0.0;
    /** Paired-servo preload: added to t1 and subtracted from t2. */
    public static double TURRET_SERVO_INTERLOCK_OFFSET = 0.0035;
    /** Robot-center to turret-center distance; positive means behind the robot. */
    public static double TURRET_REAR_OFFSET_IN = 1.77;
    public static double TURRET_ANGLE_SIGN = 1.0;
    public static double TURRET_MIN_SAFE_ANGLE = -180.0;
    public static double TURRET_MAX_SAFE_ANGLE = 180.0;

    // Aiming behavior.
    public static double OMEGA_FEEDFORWARD_GAIN = 1.0;
    /** Friend-code angular-velocity compensation time. */
    public static double SERVO_RESPONSE_LATENCY = 0.1;
    public static double OMEGA_FILTER_ALPHA = 0.7;
    /** Smoothing time constant (s); ~0.04 = smooth but quick at 50 Hz. */
    public static double SERVO_FILTER_TAU_S = 0.04;
    /** Prevents a wrap crossing from commanding almost the full servo travel at once. */
    public static double TURRET_MAX_SLEW_RATE_DEG_PER_SEC = 540.0;
    /** Hold the nearest endpoint for small unreachable crossings past +/-180 degrees. */
    public static double TURRET_WRAP_HOLD_HYSTERESIS_DEG = 15.0;
    /** Caps the time used by the slew limiter after a delayed robot loop. */
    public static double TURRET_CONTROL_MAX_DT_S = 0.05;

    // One servo pulse step (1 us over the 500-2500 us range).
    private static final double SERVO_POSITION_QUANTUM = 1.0 / 2000.0;

    /** What the aim loop produced this iteration — feed telemetry with it. */
    public static class AimResult {
        public boolean hasTarget;
        public double distance;
        public double targetTurretAngleDeg;
        public double turretAngleDeg;
        public double aimErrorDeg;
        public boolean wrapProtected;
    }

    private ServoImplEx leftturret, rightturret;
    private double filteredOmega;
    private boolean omegaFilterInitialized;
    private double filteredTargetAngle;
    private long lastUpdateNanos;
    private double lastServoPosition = 0.5;
    private double lastServo1Position = 0.5;
    private double lastServo2Position = 0.5;
    private double lastWrittenServo1Position = Double.NaN;
    private double lastWrittenServo2Position = Double.NaN;

    public void init(HardwareMap hardwareMap) {
        leftturret = hardwareMap.get(ServoImplEx.class, "leftturret");
        rightturret = hardwareMap.get(ServoImplEx.class, "rightturret");
        PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
        leftturret.setPwmRange(range);
        rightturret.setPwmRange(range);
        leftturret.setDirection(Servo.Direction.FORWARD);
        rightturret.setDirection(Servo.Direction.FORWARD);
        omegaFilterInitialized = false;
        center();
    }

    /** 目标相对车身的几何角度，单位弧度；不含旋转速度补偿。 */
    public double target() {
        Pose goal = Shoot.target();
        double headingDeg = Shoot.pp.getHeading(AngleUnit.DEGREES);
        double x = turretX(Shoot.pp.getPosX(DistanceUnit.INCH), headingDeg);
        double y = turretY(Shoot.pp.getPosY(DistanceUnit.INCH), headingDeg);
        double absoluteYaw = Math.toDegrees(Math.atan2(goal.getY() - y, goal.getX() - x));
        return Math.toRadians(normalizeDeg(absoluteYaw - headingDeg));
    }

    /** 外部每轮先刷新 Pinpoint；这里复用 Shoot 的当前定位和选框逻辑。 */
    public AimResult update() {
        Pose goal = Shoot.target();
        return update(
                Shoot.pp.getPosX(DistanceUnit.INCH),
                Shoot.pp.getPosY(DistanceUnit.INCH),
                Shoot.pp.getHeading(AngleUnit.DEGREES),
                Shoot.pp.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES),
                goal.getX(), goal.getY(), 0.0);
    }

    /**
     * Aims the turret at (targetX, targetY). Call once per loop.
     *
     * @param headingDeg       robot heading, degrees
     * @param omegaDegPerSec   robot angular velocity, degrees/second
     * @param yawOffsetDeg     driver trim added to the aim angle
     */
    public AimResult update(double robotX, double robotY, double headingDeg,
                            double omegaDegPerSec, double targetX, double targetY,
                            double yawOffsetDeg) {
        AimResult result = new AimResult();
        double dt = calculateDtSeconds();

        // Pinpoint reports the robot center. Rotate the rearward robot-frame
        // offset into field coordinates to aim from the actual turret pivot.
        double turretX = turretX(robotX, headingDeg);
        double turretY = turretY(robotY, headingDeg);

        double targetVectorX = targetX - turretX;
        double targetVectorY = targetY - turretY;
        result.distance = Math.hypot(targetVectorX, targetVectorY);
        if (!Double.isFinite(result.distance) || result.distance < 0.1) {
            center();
            return result;
        }

        if (!omegaFilterInitialized) {
            filteredOmega = omegaDegPerSec;
            omegaFilterInitialized = true;
        } else {
            filteredOmega += OMEGA_FILTER_ALPHA * (omegaDegPerSec - filteredOmega);
        }

        double absoluteYaw = Math.toDegrees(
                Math.atan2(targetVectorY, targetVectorX));
        double relativeAngle = normalizeDeg(
                absoluteYaw + yawOffsetDeg - normalizeDeg(headingDeg));
        // Lead the servo by the robot's rotation over its response latency.
        double feedforward = -OMEGA_FEEDFORWARD_GAIN * filteredOmega * SERVO_RESPONSE_LATENCY;
        double targetAngle = normalizeDeg(relativeAngle + feedforward);
        result.targetTurretAngleDeg = targetAngle;

        if (!isAngleSafe(targetAngle)) {
            center();
            return result;
        }

        double controlDt = dt > 0
                ? Math.min(dt, Math.max(0.001, TURRET_CONTROL_MAX_DT_S))
                : 0.02;
        double nearestEquivalent = filteredTargetAngle
                + normalizeDeg(targetAngle - filteredTargetAngle);
        result.wrapProtected = nearestEquivalent < TURRET_MIN_SAFE_ANGLE
                || nearestEquivalent > TURRET_MAX_SAFE_ANGLE;
        double servoReachableTarget = chooseServoReachableTarget(
                targetAngle, nearestEquivalent);

        double alpha = SERVO_FILTER_TAU_S > 0
                ? 1.0 - Math.exp(-controlDt / SERVO_FILTER_TAU_S) : 1.0;
        double filteredStep = (servoReachableTarget - filteredTargetAngle) * alpha;
        double maxStep = Math.max(0.0, TURRET_MAX_SLEW_RATE_DEG_PER_SEC) * controlDt;
        filteredTargetAngle = Range.clip(
                filteredTargetAngle + Range.clip(filteredStep, -maxStep, maxStep),
                TURRET_MIN_SAFE_ANGLE, TURRET_MAX_SAFE_ANGLE);

        result.hasTarget = true;
        result.turretAngleDeg = filteredTargetAngle;
        result.aimErrorDeg = normalizeDeg(targetAngle - filteredTargetAngle);
        commandTurretAngle(filteredTargetAngle);
        return result;
    }

    public void center() {
        filteredTargetAngle = 0;
        // Keep the smoothing time base current while a center override is held
        // so auto-aim resumes smoothly when the override is released.
        lastUpdateNanos = System.nanoTime();
        commandTurretAngle(0);
    }

    /**
     * Immediately holds a fixed robot-relative turret angle. The normal paired-
     * servo interlock preload is applied by {@link #commandTurretAngle(double)}.
     */
    public void lockTurretAtAngle(double angleDeg) {
        if (!isAngleSafe(angleDeg)) {
            center();
            return;
        }

        filteredTargetAngle = angleDeg;
        omegaFilterInitialized = false;
        lastUpdateNanos = System.nanoTime();
        commandTurretAngle(angleDeg);
    }

    public double getLastServoPosition() {
        return lastServoPosition;
    }

    public double getLastServo1Position() {
        return lastServo1Position;
    }

    public double getLastServo2Position() {
        return lastServo2Position;
    }

    private void commandTurretAngle(double angleDeg) {
        double basePosition = 0.5
                + TURRET_ANGLE_SIGN * angleDeg * TURRET_GEAR_RATIO / TURRET_SERVO_RANGE_DEG
                + TURRET_SERVO_OFFSET;
        lastServoPosition = quantize(Range.clip(basePosition, 0.0, 1.0));
        lastServo1Position = quantize(Range.clip(
                basePosition + TURRET_SERVO_INTERLOCK_OFFSET, 0.0, 1.0));
        lastServo2Position = quantize(Range.clip(
                basePosition - TURRET_SERVO_INTERLOCK_OFFSET, 0.0, 1.0));

        if (lastServo1Position != lastWrittenServo1Position) {
            leftturret.setPosition(lastServo1Position);
            lastWrittenServo1Position = lastServo1Position;
        }
        if (lastServo2Position != lastWrittenServo2Position) {
            rightturret.setPosition(lastServo2Position);
            lastWrittenServo2Position = lastServo2Position;
        }
    }

    private static boolean isAngleSafe(double angleDeg) {
        double basePosition = 0.5
                + TURRET_ANGLE_SIGN * angleDeg * TURRET_GEAR_RATIO / TURRET_SERVO_RANGE_DEG
                + TURRET_SERVO_OFFSET;
        double servo1Position = basePosition + TURRET_SERVO_INTERLOCK_OFFSET;
        double servo2Position = basePosition - TURRET_SERVO_INTERLOCK_OFFSET;
        return Double.isFinite(angleDeg)
                && angleDeg >= TURRET_MIN_SAFE_ANGLE
                && angleDeg <= TURRET_MAX_SAFE_ANGLE
                && servo1Position >= 0.01
                && servo1Position <= 0.99
                && servo2Position >= 0.01
                && servo2Position <= 0.99;
    }

    private static double quantize(double position) {
        return Math.round(position / SERVO_POSITION_QUANTUM) * SERVO_POSITION_QUANTUM;
    }

    /**
     * Selects a physical servo target for a circular angle. Small crossings
     * just beyond a mechanical endpoint hold that endpoint. Once the target is
     * farther past the endpoint than the hysteresis, the turret takes the long
     * but reachable path through its range; the slew limiter controls that move.
     */
    private static double chooseServoReachableTarget(double wrappedTarget,
                                                     double nearestEquivalent) {
        double hysteresis = Math.max(0.0, TURRET_WRAP_HOLD_HYSTERESIS_DEG);
        if (nearestEquivalent > TURRET_MAX_SAFE_ANGLE) {
            if (nearestEquivalent - TURRET_MAX_SAFE_ANGLE <= hysteresis) {
                return TURRET_MAX_SAFE_ANGLE;
            }
            return wrappedTarget;
        }
        if (nearestEquivalent < TURRET_MIN_SAFE_ANGLE) {
            if (TURRET_MIN_SAFE_ANGLE - nearestEquivalent <= hysteresis) {
                return TURRET_MIN_SAFE_ANGLE;
            }
            return wrappedTarget;
        }
        return nearestEquivalent;
    }

    public static double turretX(double robotX, double headingDeg) {
        return robotX - TURRET_REAR_OFFSET_IN * Math.cos(Math.toRadians(headingDeg));
    }

    public static double turretY(double robotY, double headingDeg) {
        return robotY - TURRET_REAR_OFFSET_IN * Math.sin(Math.toRadians(headingDeg));
    }

    private double calculateDtSeconds() {
        long now = System.nanoTime();
        double dt = lastUpdateNanos == 0L ? 0.0 : (now - lastUpdateNanos) / 1e9;
        lastUpdateNanos = now;
        return dt;
    }

    private static double normalizeDeg(double angleDeg) {
        if (!Double.isFinite(angleDeg)) {
            return angleDeg;
        }
        angleDeg %= 360.0;
        if (angleDeg > 180.0) angleDeg -= 360.0;
        if (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }
}
