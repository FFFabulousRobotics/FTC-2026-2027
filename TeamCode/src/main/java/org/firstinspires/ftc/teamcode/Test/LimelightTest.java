package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Limelight.Limelight;

import java.util.List;
import java.util.Locale;

/*Limelight_Test仅作为测试limelight是否在正常且正确地工作的工具（原来的LimelightDiag已经并进这里）
 * 先看"状态"那一行，它把故障分成三种：
 *  相机没连上：硬件或配置的问题——12V供电、网线有没有插在Control Hub的以太网口、config里的设备名是不是limelight
 *  连通但没识别到tag：打开相机网页(172.29.0.1:5801)对着tag看。网页里也没有框，就去查0号管线是不是AprilTag类型、
 *                    family是不是36h11、镜头对焦、tag够不够大够不够清楚；网页里有框而这里没有，才是代码这边的问题
 *  识别正常：下面几行是具体数值
 * 相机在INIT阶段就启动了，所以不用按PLAY，初始化页面上就能看到状态*/
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends OpMode {
    private Limelight vision;

    @Override public void init()      { vision = new Limelight(this); vision.start(); }
    @Override public void init_loop() { show(); }
    @Override public void loop()      { show(); }
    @Override public void stop()      { vision.stop(); }

    private void show() {
        vision.update();
        List<Integer> tags = vision.getTagIds();

        if (!vision.isConnected()) {
            /*"连上了"指最近250ms内控制Hub收到过相机的应答，这是SDK自己判断的，不经过封装里的任何计算*/
            telemetry.addData("状态", "相机没连上 —— 查12V供电/网线/驱动配置里的设备名");
        } else if (tags.isEmpty()) {
            telemetry.addData("状态", "相机连通，但没识别到tag —— 先看相机网页里有没有框(说明见代码开头)");
        } else {
            telemetry.addData("状态", "识别正常");
            telemetry.addData("Tag", vision.getTagId());
            telemetry.addData("X", vision.getTx());
            telemetry.addData("Y", vision.getTy());
            telemetry.addData("视野里的tag", "%d个 %s", tags.size(), tags);
            telemetry.addData("ta", vision.getTa());
            if (vision.getBotPose() != null) {
                telemetry.addData("场地X/Y/朝向", String.format(Locale.ROOT, "%.1f  %.1f  %.1f",
                        vision.getFieldX(), vision.getFieldY(), vision.getFieldHeading()));
            }
        }
        telemetry.update();
    }
}
