package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Auto.Position;

@Configurable
public class Shoot {
    // 今年 Shoot Encoder Test 实测参数；转速 ticks/s，积分按秒累计。
    public static double P = 0.00018194;
    public static double I = 0.00021886;
    public static double D = 0.00000000;
    public static double F = 0.00036389;
    public static final double MAX_TARGET_VELOCITY_TICKS_S = 1900.0;
    // 当前背板初始位置，正式程序和测试共用。
    public static final double BOARD_INITIAL_POSITION = 0.399;
    public static final double GATE_OPEN_POSITION = 0.3;
    public static final double GATE_CLOSED_POSITION = 0.0;
    // distance (inch)、target (ticks/s)、board；前五行为 Shoot Test 反复命中的标定点。
    // 转速与背板成对插值；100 inch 端点按末段趋势外推，尚未实测。
    // 低于 19.02 inch 或高于 100 inch 时使用最近端点。
    private static final double[][] SHOT_CALIBRATION = {
            {19.02, 1550.0, 0.059},
            {32.23, 1650.0, 0.399},
            {35.97, 1700.0, 0.369},
            {43.06, 1550.0, 0.399},
            {64.38, 1550.0, 0.409},
            {100.0, 1550.0, 0.42570732}
    };
    public static GoBildaPinpointDriver pp;
    private DcMotorEx leftflywheel, rightflywheel;
    private double targetVelocity, measuredVelocity, flywheelPower;
    private double integralPower, previousVelocity, filteredDerivative;
    private double previousTarget = -1;
    private long lastVelocityUpdateNanos;
    private ServoImplEx board;
    private ServoImplEx gate;
    private Turret turret = new Turret();
    private MovingShotController movingShotController = new MovingShotController();
    // 两个目标分别保留人工修正；自动结束时可接续给手动，不修改坐标。
    private double[] turretYawTrimDeg = new double[2];
    private static AimHandoff pendingAimHandoff;
    private static Pose handedOffGoal;
    private static boolean handoffRightRegion;
    private static class AimHandoff {
        final Turret turret;
        final MovingShotController movingShotController;
        final double[] trim;
        final Pose target;
        final boolean rightRegion;
        AimHandoff(Shoot shoot) {
            turret = shoot.turret;
            movingShotController = shoot.movingShotController;
            trim = shoot.turretYawTrimDeg;
            target = goal;
            rightRegion = pp.getPosX(DistanceUnit.INCH) > Position.MIDDLE;
        }
    }
    private boolean fixedShotSettings;
    private boolean fixedTurret;
    private boolean turretLockedAtCenter;
    private double fixedShotVelocity;
    private double fixedBoardPosition = Double.NaN;
    public static Pose goal;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    /** Static Manual：背板不初始化，云台两个舵机上电锁在 0.5。 */
    public void initStatic(HardwareMap hardwareMap) {
        turretLockedAtCenter = true;
        init(hardwareMap, false, false);
    }

    public void init(HardwareMap hardwareMap, boolean restoreAutoAim) {
        init(hardwareMap, restoreAutoAim, true);
    }

    public void init(HardwareMap hardwareMap, boolean restoreAutoAim, boolean initializeBoard) {
        initFlywheels(hardwareMap);
        board = hardwareMap.get(ServoImplEx.class, "board");
        if (initializeBoard) initBoard(board);
        gate = hardwareMap.get(ServoImplEx.class, "gate");
        gate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gateClose();
        initAiming(hardwareMap, restoreAutoAim);
    }

    /** 只绑定 PP 和炮塔；LEAVE 自动不需要初始化飞轮、背板或进球机构。 */
    public void initAiming(HardwareMap hardwareMap, boolean restoreAutoAim) {
        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        AimHandoff handoff = restoreAutoAim ? pendingAimHandoff : null;
        pendingAimHandoff = null;
        handedOffGoal = handoff == null ? null : handoff.target;
        if (handoff != null) {
            handoffRightRegion = handoff.rightRegion;
            turret = handoff.turret;
            movingShotController = handoff.movingShotController;
            turretYawTrimDeg = handoff.trim;
        }
        if (turretLockedAtCenter) turret.initLocked(hardwareMap);
        else turret.init(hardwareMap, handoff != null);
    }

    /** RC 进程内一次性接续，PP 仍保留硬件上的实际场地坐标。 */
    public void saveAimingForTeleOp() {
        pendingAimHandoff = new AimHandoff(this);
    }

    /** 供独立调速测试使用，仅初始化两个飞轮。 */
    public void initFlywheels(HardwareMap hardwareMap) {
        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        leftflywheel.setPower(0);
        leftflywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        rightflywheel.setPower(0);
        rightflywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        targetVelocity = 0;
        measuredVelocity = 0;
        flywheelPower = 0;
        resetVelocityControl();
    }

    public static void initBoard(ServoImplEx board) {
        board.setPwmRange(new PwmControl.PwmRange(500, 2500));
        board.setPosition(BOARD_INITIAL_POSITION);
    }

    /** 手动模式初始化/开始时，按当前定位到所选射击框的距离设置背板。 */
    public double prepareBoardForCurrentTarget() {
        if (fixedShotSettings) {
            if (!Double.isFinite(fixedBoardPosition)) return board.getPosition();
            board.setPosition(fixedBoardPosition);
            return fixedBoardPosition;
        }
        target();
        return angle(distance());
    }

    /** 仅当前 Shoot 实例使用固定射击参数，炮塔仍按原逻辑瞄准和人工修正。 */
    public void useFixedShotSettings(double velocityTicksPerSecond, double boardPosition) {
        fixedShotSettings = true;
        fixedShotVelocity = Range.clip(velocityTicksPerSecond, 0, MAX_TARGET_VELOCITY_TICKS_S);
        fixedBoardPosition = Range.clip(boardPosition, 0, 1);
    }

    public void useFixedShotVelocity(double velocityTicksPerSecond) {
        fixedShotSettings = true;
        fixedShotVelocity = Range.clip(velocityTicksPerSecond, 0, MAX_TARGET_VELOCITY_TICKS_S);
    }

    public void useFixedTurret() {
        fixedTurret = true;
    }

    public double getBoardPosition() {
        return board.getPosition();
    }

    public void gateOpen() {
        gate.setPosition(GATE_OPEN_POSITION);
    }

    public void gateClose() {
        gate.setPosition(GATE_CLOSED_POSITION);
    }

    public void stop() {
        leftflywheel.setPower(0);
        rightflywheel.setPower(0);
        targetVelocity = 0;
        flywheelPower = 0;
        resetVelocityControl();
        gateClose();
    }

    public static Pose target() {
        boolean rightRegion = pp.getPosX(DistanceUnit.INCH) > Position.MIDDLE;
        // 接手时沿用自动最后的目标；跨过原来的分区线后恢复原选框规则。
        if (handedOffGoal != null && rightRegion == handoffRightRegion) {
            goal = handedOffGoal;
            return goal;
        }
        handedOffGoal = null;
        if(rightRegion) goal = Position.SHOOT_POSE_2;
        else goal = Position.SHOOT_POSE_1;
        return goal;
    }

    public static double distance() {
        double CurrentX = pp.getPosX(DistanceUnit.INCH);
        double CurrentY = pp.getPosY(DistanceUnit.INCH);
        double DeltaX = CurrentX - goal.getX();
        double DeltaY = CurrentY - goal.getY();
        return Math.sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
    }
    public double velocity() {
        return velocity(distance());
    }

    private double velocity(double distance) {
        setVelocity(interpolateShot(distance, 1));
        return targetVelocity;
    }

    private static double interpolateShot(double distance, int column) {
        // 第三段 35.97～43.06 inch（含两端）：飞轮 +20；背板 +0.01 后再减 0.015，净 -0.005。
        double adjustment = distance >= 35.97 && distance <= 43.06
                ? (column == 1 ? 20.0 : -0.005) : 0.0;
        if (distance <= SHOT_CALIBRATION[0][0]) return SHOT_CALIBRATION[0][column] + adjustment;
        for (int i = 1; i < SHOT_CALIBRATION.length; i++) {
            double[] lower = SHOT_CALIBRATION[i - 1];
            double[] upper = SHOT_CALIBRATION[i];
            if (distance <= upper[0]) {
                double fraction = (distance - lower[0]) / (upper[0] - lower[0]);
                return lower[column] + fraction * (upper[column] - lower[column]) + adjustment;
            }
        }
        return SHOT_CALIBRATION[SHOT_CALIBRATION.length - 1][column] + adjustment;
    }

    /** 每轮调用：只读取右飞轮编码器，用同一份功率驱动两个飞轮。 */
    public void setVelocity(double ticksPerSecond) {
        long now = System.nanoTime();
        double dt = lastVelocityUpdateNanos == 0 ? 0.001
                : Range.clip((now - lastVelocityUpdateNanos) / 1e9, 0.001, 0.1);
        lastVelocityUpdateNanos = now;
        targetVelocity = Range.clip(ticksPerSecond, 0.0, MAX_TARGET_VELOCITY_TICKS_S);
        measuredVelocity = Math.abs(rightflywheel.getVelocity());
        if (targetVelocity != previousTarget) {
            previousTarget = targetVelocity;
            previousVelocity = measuredVelocity;
            filteredDerivative = 0;
        }
        if (targetVelocity < 1.0) {
            integralPower = filteredDerivative = flywheelPower = 0;
            previousVelocity = measuredVelocity;
        } else {
            double error = targetVelocity - measuredVelocity;
            // 与标定测试相同：D 取实测速度负导数并滤波。
            filteredDerivative += dt / (0.05 + dt)
                    * (-(measuredVelocity - previousVelocity) / dt - filteredDerivative);
            previousVelocity = measuredVelocity;
            double feedAndFeedback = F * targetVelocity + P * error + D * filteredDerivative;
            double candidate = Range.clip(integralPower + I * error * dt, -1, 1);
            double raw = feedAndFeedback + candidate;
            // 输出饱和时，仅允许积分向解除饱和的方向变化。
            if ((raw >= 0 && raw <= 1) || (raw > 1 && error < 0) || (raw < 0 && error > 0)) {
                integralPower = candidate;
            }
            flywheelPower = Range.clip(feedAndFeedback + integralPower, 0, 1);
        }
        leftflywheel.setPower(flywheelPower);
        rightflywheel.setPower(flywheelPower);
    }

    private void resetVelocityControl() {
        integralPower = filteredDerivative = 0;
        previousVelocity = measuredVelocity;
        previousTarget = -1;
        lastVelocityUpdateNanos = 0;
    }

    public double getMeasuredVelocity() {
        return measuredVelocity;
    }

    public double getPower() {
        return flywheelPower;
    }

    private double angle(double distance) {
        double angle = interpolateShot(distance, 2);
        board.setPosition(angle);
        return angle;
    }
    public void update(Telemetry telemetry) {
        update(telemetry, target());
    }

    public void update(Telemetry telemetry, Pose target) {
        update(telemetry, target, 0.0, false);
    }

    /** 正式手动模式：Y 逆时针、B 顺时针；同时按下清除两个目标的人工修正。 */
    public void update(Telemetry telemetry, boolean turretLeft, boolean turretRight) {
        update(telemetry, target(), (turretLeft ? 1.0 : 0.0) - (turretRight ? 1.0 : 0.0),
                turretLeft && turretRight);
    }

    private void update(Telemetry telemetry, Pose target, double manualDirection, boolean clearTrim) {
        MovingShotController.Result movingShot = updateAiming(telemetry, target, manualDirection, clearTrim);
        double targetVelocity;
        double boardPosition;
        if (fixedShotSettings) {
            setVelocity(fixedShotVelocity);
            targetVelocity = fixedShotVelocity;
            boardPosition = Double.isFinite(fixedBoardPosition) ? fixedBoardPosition : board.getPosition();
            if (Double.isFinite(fixedBoardPosition)) board.setPosition(boardPosition);
        } else {
            targetVelocity = velocity(movingShot.shootDistance);
            boardPosition = angle(movingShot.shootDistance);
        }
        telemetry.addData("velocity", targetVelocity);
        telemetry.addData("right measured velocity (ticks/s)", measuredVelocity);
        telemetry.addData("flywheel power (both)", flywheelPower);
        telemetry.addData("angle", boardPosition);
    }

    /** 外部先更新定位；仅对准并添加 telemetry，绝不启动飞轮或进球。 */
    public void updateAiming(Telemetry telemetry) {
        updateAiming(telemetry, target(), 0.0, false);
    }

    private MovingShotController.Result updateAiming(Telemetry telemetry, Pose target,
                                                     double manualDirection, boolean clearTrim) {
        goal = target;
        int trimIndex = target == Position.SHOOT_POSE_2 ? 1 : 0;
        if (clearTrim) {
            turretYawTrimDeg[0] = turretYawTrimDeg[1] = 0.0;
        }
        // 外部每轮先 pp.update()，这里统一使用这一次读取的定位数据。
        double robotX = pp.getPosX(DistanceUnit.INCH);
        double robotY = pp.getPosY(DistanceUnit.INCH);
        double heading = pp.getHeading(AngleUnit.DEGREES);
        double omega = pp.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        // 与 Shoot Test 标定一致，查表距离从 Pinpoint 参考点计算。
        // Turret.update 内部仍使用炮塔轴心偏移瞄准。
        MovingShotController.Result movingShot = movingShotController.update(
                robotX, robotY, goal.getX(), goal.getY(),
                pp.getVelX(DistanceUnit.INCH), pp.getVelY(DistanceUnit.INCH));
        Turret.AimResult aim;
        if (turretLockedAtCenter) {
            turret.holdServoCenter();
            aim = new Turret.AimResult();
        } else if (fixedTurret) {
            if (clearTrim) turret.center();
            aim = turret.updateManual(manualDirection);
        } else {
            aim = turret.update(robotX, robotY, heading, omega,
                    movingShot.aimTargetX, movingShot.aimTargetY,
                    turretYawTrimDeg[trimIndex], manualDirection);
        }
        if (aim.manualOverride && aim.hasTarget) {
            turretYawTrimDeg[trimIndex] = aim.yawOffsetDeg;
        }
        telemetry.addData("distance (in)", Math.hypot(goal.getX() - robotX, goal.getY() - robotY));
        telemetry.addData("shot distance (in)", movingShot.shootDistance);
        telemetry.addData("moving shot", movingShot.active ? "ON" : "STATIC");
        telemetry.addData("moving velocity (in/s)", "%.1f, %.1f",
                movingShot.usedVelocityX, movingShot.usedVelocityY);
        telemetry.addData("moving correction (deg)", movingShot.headingCorrectionDeg);
        telemetry.addData("turret (deg)", "target %.1f | command %.1f",
                aim.targetTurretAngleDeg, aim.turretAngleDeg);
        telemetry.addData("turret control", turretLockedAtCenter ? "LOCKED 0.500 / 0.500"
                : (fixedTurret ? "STATIC Y/B" : (aim.manualOverride ? "MANUAL Y/B" : "AUTO + TRIM")));
        telemetry.addData("turret trim (deg)", "goal %d: %+.2f | 1 %+.2f / 2 %+.2f",
                trimIndex + 1, turretYawTrimDeg[trimIndex], turretYawTrimDeg[0], turretYawTrimDeg[1]);
        // 只添加射击数据，由调用方统一刷新，避免底盘与射击信息交替覆盖。
        return movingShot;
    }
}
