package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PinpointLocalization {
    private GoBildaPinpointDriver pinpoint;
    private Pose2D initialPose;
    private boolean initialPosePending;

    /** 手动/测试入口：配置定位参数，保留 Pinpoint 当前的场地坐标和朝向。 */
    public void init(HardwareMap hardwareMap) {
        PinpointConstants config = Constants.pinpointConstants;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.hardwareMapName);
        pinpoint.setOffsets(config.forwardPodY, config.strafePodX, config.distanceUnit);
        if (config.customEncoderResolution.isPresent()) {
            pinpoint.setEncoderResolution(config.customEncoderResolution.getAsDouble(), config.distanceUnit);
        } else {
            pinpoint.setEncoderResolution(config.encoderResolution);
        }
        pinpoint.setEncoderDirections(config.forwardEncoderDirection, config.strafeEncoderDirection);
        if (config.yawScalar.isPresent()) pinpoint.setYawScalar(config.yawScalar.getAsDouble());
        initialPosePending = false;
    }

    /** 自动入口：机器人静止时校准；init_loop 持续调用 update，等 isReady 后开始。 */
    public void initAuto(HardwareMap hardwareMap, Pose startPose) {
        init(hardwareMap);
        initialPose = new Pose2D(DistanceUnit.INCH, startPose.getX(), startPose.getY(),
                AngleUnit.RADIANS, startPose.getHeading());
        pinpoint.resetPosAndIMU();
        initialPosePending = true;
    }

    /** 每轮读取一次；首次校准完成后写入起点并立即读回。 */
    public Pose2D update() {
        pinpoint.update();
        if (initialPosePending
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            pinpoint.setPosition(initialPose);
            initialPosePending = false;
            pinpoint.update();
        }
        return pinpoint.getPosition();
    }

    public boolean isReady() {
        return !initialPosePending
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }
}
