package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*LimelightDiag是LimelightTest的加强版，用来区分"SDK和相机没通信"与"相机在通信但没识别到目标"这两种情况
 * LimelightTest里hasTarget=false可能意味着上面任意一种，这个类会把失败原因具体打印出来
 * 使用时配合limelight本地网页(172.29.0.1:5801)一起看：网页里能看到绿框但这里无目标，才是代码/SDK侧的问题*/
@TeleOp(name = "Limelight Diag", group = "Test")
public class LimelightDiag extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    /*故意不经过Limelight封装类，直接操作Limelight3A，这样封装类里的任何问题都不会干扰诊断*/
    @Override public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();//放在init里而不是start里，让相机在INIT阶段就预热，按下PLAY立刻有结果

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
    }

    @Override public void loop() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
        LLResult r = limelight.getLatestResult();

        if (r == null) {
            /*连LLResult对象都拿不到，说明SDK和相机之间根本没有数据流动
             *查：12V供电、网线是否插在Control Hub的以太网口、驱动配置里设备名是否为limelight*/
            telemetry.addData("状态", "LLResult为null —— SDK与相机无通信，查供电/网线/驱动配置");
        } else if (!r.isValid()) {
            /*能拿到结果对象但valid为false，说明通信正常，只是pipeline一帧目标都没识别到
             *查：0号管道类型是否为AprilTag(网页端Input页)、family是否36h11、镜头对焦/保护膜、
             *画面里是否真的有打印清晰的36h11 apriltag(距镜头0.5~2m为宜)*/
            telemetry.addData("状态", "通信正常但无目标 —— 查pipeline类型/对焦/画面里是否真有tag");
        } else {
            telemetry.addData("状态", "识别正常");
            telemetry.addData("tx/ty/ta", String.format("%.2f  %.2f  %.2f", r.getTx(), r.getTy(), r.getTa()));
            List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
            telemetry.addData("tag数量", tags == null ? 0 : tags.size());
            if (tags != null && !tags.isEmpty()) {
                for (LLResultTypes.FiducialResult t : tags) {
                    telemetry.addData("tag " + t.getFiducialId(), "在视野内");
                }
                Pose3D pose = r.getBotpose_MT2();
                if (pose != null) {
                    telemetry.addData("场地X/Y/朝向", String.format("%.1f  %.1f  %.1f",
                            pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw()));
                }
            }
        }
        telemetry.update();
    }

    @Override public void stop() { limelight.stop(); }
}
