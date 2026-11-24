package org.firstinspires.ftc.teamcode.ManualOpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.LiftAndNet;
import org.firstinspires.ftc.teamcode.PinpointLocalization;

@TeleOp(name = "Manual Mecanum Drive", group = "Manual")
public class ManualMecanumDrive extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private final LiftAndNet liftAndNet = new LiftAndNet();
    private final PinpointLocalization localization = new PinpointLocalization();

    protected boolean isFieldCentric() {
        return false;
    }

    @Override
    public void runOpMode() {
        // initialize
        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        leftBack   = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack  = hardwareMap.get(DcMotor.class, "rb");

        //Reverse
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftAndNet.init(hardwareMap);
        localization.init(hardwareMap);

        IMU imu = null;
        if (isFieldCentric()) {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        }

        telemetry.addData("Status", "已初始化，等待开始");
        telemetry.addLine(imu == null ? "有头模式" : "无头模式：A 重设前方");
        telemetry.addLine("L1 抬升，L2 下降，松手停止");
        telemetry.addLine("X 切换 net");
        telemetry.update();

        waitForStart();
        if (imu != null) imu.resetYaw();

        while (opModeIsActive()) {
            Pose2D pose = localization.update();
            liftAndNet.update(gamepad1.left_bumper, gamepad1.left_trigger > 0.1);
            if (gamepad1.xWasPressed()) liftAndNet.toggleNet();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            if (imu != null) {
                if (gamepad1.aWasPressed()) imu.resetYaw();
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double cos = Math.cos(-heading);
                double sin = Math.sin(-heading);
                double robotX = x * cos - y * sin;
                y = x * sin + y * cos;
                x = robotX;
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            }

            double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            //Setpower
            leftFront.setPower((y + x + rx) / d);
            rightFront.setPower((y - x - rx) / d);
            leftBack.setPower((y - x + rx) / d);
            rightBack.setPower((y + x - rx) / d);

            telemetry.addData("Forward y", "%.2f", y);
            telemetry.addData("Strafe x", "%.2f", x);
            telemetry.addData("Rotate rx", "%.2f", rx);
            telemetry.addData("Pose (in, deg)", "%.2f, %.2f, %.1f",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint", localization.getDriver().getDeviceStatus());
            telemetry.update();
        }
        liftAndNet.stop();
    }
}
