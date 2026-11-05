package org.firstinspires.ftc.teamcode.manualopmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Manual Mecanum Drive", group = "Manual")
public class ManualMecanumDrive extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    @Override
    public void runOpMode() {
        // initialize
        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        leftBack   = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack  = hardwareMap.get(DcMotor.class, "rb");

        //Reverse
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "已初始化，等待开始");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            // gamepad
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            //Setpower
            leftFront.setPower((y + x + rx) / d);
            rightFront.setPower((y - x - rx) / d);
            leftBack.setPower((y - x + rx) / d);
            rightBack.setPower((y + x - rx) / d);

            telemetry.addData("Forward y", "%.2f", y);
            telemetry.addData("Strafe x", "%.2f", x);
            telemetry.addData("Rotate rx", "%.2f", rx);
            telemetry.update();
        }
    }
}
