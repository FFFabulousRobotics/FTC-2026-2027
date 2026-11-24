package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * Shared shooting-while-moving solution ported from 19666 Good_Tele and
 * adapted to produce a field target for this robot's turret.
 *
 * One instance belongs to each running OpMode. The Result object is reused so
 * the robot loop does not allocate garbage every iteration.
 */
@Configurable
public final class MovingShotController {
    public static boolean MOVING_SHOT_ENABLED = true;
    public static boolean MOVING_SHOT_DISTANCE_COMPENSATION_ENABLED = true;
    // Ported tuning values: recalibrate lead times and speed limits on this robot.
    public static double MOVING_SHOT_HEADING_LEAD_TIME_SEC = 8.0;
    public static double MOVING_SHOT_DISTANCE_LEAD_TIME_SEC = 3.0;
    public static double MOVING_SHOT_COMMAND_BLEND = 0.35;
    public static double MOVING_SHOT_SHOOTER_READY_TOLERANCE = 0.04;
    public static double MOVING_SHOT_TARGET_FILTER_ALPHA = 0.40;
    public static double MOVING_SHOT_MAX_TARGET_STEP_DEG = 3.0;
    public static double MOVING_SHOT_PP_VELOCITY_FILTER_ALPHA = 0.35;
    public static double MOVING_SHOT_MAX_COMMAND_VELOCITY_IPS = 60.0;
    public static double MOVING_SHOT_MIN_SPEED_IPS = 3.0;
    public static double MOVING_SHOT_MAX_LEAD_IN = 36.0;
    public static double MOVING_SHOT_COMMAND_X_SIGN = 1.0;
    public static double MOVING_SHOT_COMMAND_Y_SIGN = -1.0;

    public static final class Result {
        public boolean active;
        public boolean shooterReady;
        public double rawTargetHeadingDeg;
        public double targetHeadingDeg;
        public double shootDistance;
        public double aimTargetX;
        public double aimTargetY;
        public double ppVelocityX;
        public double ppVelocityY;
        public double commandVelocityX;
        public double commandVelocityY;
        public double usedVelocityX;
        public double usedVelocityY;
        public double headingLeadDistance;
        public double distanceLeadDistance;
        public double headingCorrectionDeg;
        public double shooterError;
    }

    private final Result result = new Result();
    private double filteredVelocityX;
    private double filteredVelocityY;
    private double filteredTargetHeadingDeg;
    private boolean velocityInitialized;
    private boolean targetInitialized;

    /** Pinpoint-only field velocity, for both autonomous and TeleOp. */
    public Result update(double originX, double originY,
                         double targetX, double targetY,
                         double ppVelocityX, double ppVelocityY) {
        updateVelocity(ppVelocityX, ppVelocityY, 0.0, true, 0.0, 0.0);
        // No command estimate is available here: preserve the full measured speed.
        result.usedVelocityX = filteredVelocityX;
        result.usedVelocityY = filteredVelocityY;
        return calculate(originX, originY, targetX, targetY, targetX, targetY);
    }

    public Result update(
            double originX, double originY,
            double aimTargetX, double aimTargetY,
            double distanceTargetX, double distanceTargetY,
            double ppVelocityX, double ppVelocityY,
            double headingDeg, boolean fieldCentricEnabled,
            double leftStickY, double leftStickX) {
        updateVelocity(ppVelocityX, ppVelocityY, headingDeg,
                fieldCentricEnabled, leftStickY, leftStickX);
        return calculate(originX, originY, aimTargetX, aimTargetY,
                distanceTargetX, distanceTargetY);
    }

    private Result calculate(double originX, double originY,
                             double aimTargetX, double aimTargetY,
                             double distanceTargetX, double distanceTargetY) {
        double baseAimX = aimTargetX - originX;
        double baseAimY = aimTargetY - originY;
        double baseDistanceX = distanceTargetX - originX;
        double baseDistanceY = distanceTargetY - originY;
        double baseTargetHeadingDeg = Math.toDegrees(
                Math.atan2(baseAimY, baseAimX));
        double baseDistance = Math.hypot(baseDistanceX, baseDistanceY);

        result.rawTargetHeadingDeg = baseTargetHeadingDeg;
        result.targetHeadingDeg = baseTargetHeadingDeg;
        result.shootDistance = baseDistance;
        result.aimTargetX = aimTargetX;
        result.aimTargetY = aimTargetY;
        result.headingLeadDistance = 0.0;
        result.distanceLeadDistance = 0.0;
        result.headingCorrectionDeg = 0.0;
        result.active = MOVING_SHOT_ENABLED
                && Math.hypot(result.usedVelocityX, result.usedVelocityY)
                >= Math.max(0.0, MOVING_SHOT_MIN_SPEED_IPS);

        if (!result.active) {
            targetInitialized = false;
            return result;
        }

        double headingLeadTime = Math.max(0.0,
                MOVING_SHOT_HEADING_LEAD_TIME_SEC);
        double distanceLeadTime = Math.max(0.0,
                MOVING_SHOT_DISTANCE_LEAD_TIME_SEC);
        double headingLeadX = result.usedVelocityX * headingLeadTime;
        double headingLeadY = result.usedVelocityY * headingLeadTime;
        double distanceLeadX = result.usedVelocityX * distanceLeadTime;
        double distanceLeadY = result.usedVelocityY * distanceLeadTime;
        result.headingLeadDistance = Math.hypot(headingLeadX, headingLeadY);
        result.distanceLeadDistance = Math.hypot(distanceLeadX, distanceLeadY);

        double maxLead = Math.max(0.0, MOVING_SHOT_MAX_LEAD_IN);
        if (maxLead > 0.0 && result.headingLeadDistance > maxLead) {
            double scale = maxLead / result.headingLeadDistance;
            headingLeadX *= scale;
            headingLeadY *= scale;
            result.headingLeadDistance = maxLead;
        }
        if (maxLead > 0.0 && result.distanceLeadDistance > maxLead) {
            double scale = maxLead / result.distanceLeadDistance;
            distanceLeadX *= scale;
            distanceLeadY *= scale;
            result.distanceLeadDistance = maxLead;
        }

        double distanceCorrectedX = baseDistanceX - distanceLeadX;
        double distanceCorrectedY = baseDistanceY - distanceLeadY;
        double headingCorrectedX = baseAimX - headingLeadX;
        double headingCorrectedY = baseAimY - headingLeadY;
        result.rawTargetHeadingDeg = Math.toDegrees(
                Math.atan2(headingCorrectedY, headingCorrectedX));
        result.targetHeadingDeg = smoothTarget(result.rawTargetHeadingDeg);
        double movingDistance = Math.hypot(
                distanceCorrectedX, distanceCorrectedY);
        result.shootDistance = MOVING_SHOT_DISTANCE_COMPENSATION_ENABLED
                ? movingDistance : baseDistance;
        result.headingCorrectionDeg = wrap180(
                result.targetHeadingDeg - baseTargetHeadingDeg);

        // Turret consumes a field point. Build a virtual ray from the turret
        // pivot that preserves the smoothed absolute heading from Good_Tele.
        double aimRayDistance = Math.max(1.0,
                Math.hypot(headingCorrectedX, headingCorrectedY));
        double movingAimRadians = Math.toRadians(result.targetHeadingDeg);
        result.aimTargetX = originX
                + Math.cos(movingAimRadians) * aimRayDistance;
        result.aimTargetY = originY
                + Math.sin(movingAimRadians) * aimRayDistance;
        return result;
    }

    public boolean updateShooterReadiness(
            double targetVelocity, double measuredVelocity) {
        result.shooterError = targetVelocity - measuredVelocity;
        result.shooterReady = targetVelocity <= 0.0
                || Math.abs(result.shooterError) <= targetVelocity
                * Math.max(0.0, MOVING_SHOT_SHOOTER_READY_TOLERANCE);
        return result.shooterReady;
    }

    private void updateVelocity(
            double ppVelocityX, double ppVelocityY,
            double headingDeg, boolean fieldCentricEnabled,
            double leftStickY, double leftStickX) {
        result.ppVelocityX = ppVelocityX;
        result.ppVelocityY = ppVelocityY;

        if (!velocityInitialized) {
            filteredVelocityX = ppVelocityX;
            filteredVelocityY = ppVelocityY;
            velocityInitialized = true;
        } else {
            double alpha = Range.clip(
                    MOVING_SHOT_PP_VELOCITY_FILTER_ALPHA, 0.0, 1.0);
            filteredVelocityX = filteredVelocityX * (1.0 - alpha)
                    + ppVelocityX * alpha;
            filteredVelocityY = filteredVelocityY * (1.0 - alpha)
                    + ppVelocityY * alpha;
        }

        double maxCommandVelocity = Math.max(
                0.0, MOVING_SHOT_MAX_COMMAND_VELOCITY_IPS);
        double commandX = -leftStickY * maxCommandVelocity
                * MOVING_SHOT_COMMAND_X_SIGN;
        double commandY = leftStickX * maxCommandVelocity
                * MOVING_SHOT_COMMAND_Y_SIGN;

        // Good_Tele is field-oriented. Rotate the estimate only when this
        // robot is operating in its optional robot-centric drive mode.
        if (!fieldCentricEnabled) {
            double headingRadians = Math.toRadians(headingDeg);
            result.commandVelocityX = commandX * Math.cos(headingRadians)
                    - commandY * Math.sin(headingRadians);
            result.commandVelocityY = commandX * Math.sin(headingRadians)
                    + commandY * Math.cos(headingRadians);
        } else {
            result.commandVelocityX = commandX;
            result.commandVelocityY = commandY;
        }

        double commandBlend = Range.clip(
                MOVING_SHOT_COMMAND_BLEND, 0.0, 1.0);
        result.usedVelocityX = filteredVelocityX * (1.0 - commandBlend)
                + result.commandVelocityX * commandBlend;
        result.usedVelocityY = filteredVelocityY * (1.0 - commandBlend)
                + result.commandVelocityY * commandBlend;
    }

    private double smoothTarget(double rawTargetDeg) {
        if (!targetInitialized) {
            filteredTargetHeadingDeg = rawTargetDeg;
            targetInitialized = true;
            return rawTargetDeg;
        }

        double alpha = Range.clip(
                MOVING_SHOT_TARGET_FILTER_ALPHA, 0.0, 1.0);
        double requestedStep = wrap180(
                rawTargetDeg - filteredTargetHeadingDeg) * alpha;
        double maxStep = Math.max(0.0, MOVING_SHOT_MAX_TARGET_STEP_DEG);
        if (maxStep > 0.0) {
            requestedStep = Range.clip(requestedStep, -maxStep, maxStep);
        }
        filteredTargetHeadingDeg = wrap180(
                filteredTargetHeadingDeg + requestedStep);
        return filteredTargetHeadingDeg;
    }

    private static double wrap180(double angleDeg) {
        while (angleDeg >= 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }
}
