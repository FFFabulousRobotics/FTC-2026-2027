package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class PinpointLocalization {
    // 本赛季待标定：偏移单位为毫米，编码器型号和方向按实际安装填写。
    public static double X_POD_OFFSET_MM = 0.0;
    public static double Y_POD_OFFSET_MM = 0.0;
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private GoBildaPinpointDriver pinpoint;
    private Pose2D initialPose;
    private boolean initialPosePending;

    /** 手动/测试入口：配置定位参数，保留 Pinpoint 当前的场地坐标和朝向。 */
    public void init(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_DIRECTION, Y_DIRECTION);
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
