package org.firstinspires.ftc.teamcode.ManualOpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PinpointLocalization;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shoot;

@TeleOp(name = "ManualOpMode", group = "Manual")
public class ManualOpMode extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private final PinpointLocalization localization = new PinpointLocalization();
    private Intake intake = new Intake();
    private Shoot shoot = new Shoot();
    private ElapsedTime timer = new ElapsedTime();
    double shoottime;
    boolean ONSHOOT = false;

    protected boolean isFieldCentric() {
        return false;
    }

    protected void prepareShooter(Shoot shooter, boolean localizationReady) {
        if (localizationReady) shooter.prepareBoardForCurrentTarget();
    }

    protected void initShooter(Shoot shooter) {
        shooter.init(hardwareMap);
    }

    protected void updateShooter(Shoot shooter) {
        shooter.update(telemetry, gamepad1.y, gamepad1.b);
    }

    protected double drivePowerScale() {
        return gamepad1.xWasPressed() ? 0.5 : 1.0;
    }

    protected String turretControlHint() {
        return "炮塔：按住 Y 逆时针 / B 顺时针，松开保留修正；Y+B 清除修正";
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

        localization.init(hardwareMap);
        intake.init(hardwareMap);
        initShooter(shoot);
        shoot.gateClose();
        localization.update();
        prepareShooter(shoot, localization.isReady());


        IMU imu = null;
        if (isFieldCentric()) {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        }

        telemetry.addData("Status", "已初始化，等待开始");
        telemetry.addLine(imu == null ? "有头模式" : "无头模式：A 重设前方");
        telemetry.addLine("按住 L1 吐球，松开停止");
        telemetry.addLine(turretControlHint());
        telemetry.update();

        while (opModeInInit()) {
            localization.update();
            prepareShooter(shoot, localization.isReady());
            if (localization.isReady()) shoot.updateAiming(telemetry);
            telemetry.addData("Status", "等待开始，沿用 PP 坐标");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        localization.update();
        prepareShooter(shoot, localization.isReady());
        if (imu != null) imu.resetYaw();
        timer.reset();

        while (opModeIsActive()) {
            Pose2D pose = localization.update();

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
            double driveScale = drivePowerScale();
            leftFront.setPower((y + x + rx) / d * driveScale);
            rightFront.setPower((y - x - rx) / d * driveScale);
            leftBack.setPower((y - x + rx) / d * driveScale);
            rightBack.setPower((y + x - rx) / d * driveScale);


            telemetry.addData("Forward y", "%.2f", y);
            telemetry.addData("Strafe x", "%.2f", x);
            telemetry.addData("Rotate rx", "%.2f", rx);
            telemetry.addData("Pose (in, deg)", "%.2f, %.2f, %.1f",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint", localization.getDriver().getDeviceStatus());
            updateShooter(shoot);

            if(gamepad1.rightTriggerWasPressed() && !ONSHOOT) {
                shoottime = timer.seconds();
                shoot.gateOpen();
                ONSHOOT = true;
            }

            if (gamepad1.left_bumper) {
                intake.kick();
            } else if(ONSHOOT) {
                double deltatime = timer.seconds() - shoottime;
                if((Shoot.distance() > 115 && deltatime > 0.6) || (Shoot.distance() <= 115 && deltatime > 0.55)) {
                    intake.stop();
                    shoot.gateClose();
                    ONSHOOT = false;
                }
                else if(deltatime >= 0.2) intake.feed();
                else if(deltatime < 0.2) intake.stop();
            } else {
                if (gamepad1.left_trigger_pressed) intake.intake();
                else intake.stop();
            }
            telemetry.update();
        }
        intake.stop();
        shoot.stop();
    }
}
