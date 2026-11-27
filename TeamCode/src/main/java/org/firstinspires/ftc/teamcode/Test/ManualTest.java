package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shoot;

@Configurable
@TeleOp(name = "Manual Test", group = "Test")
public class ManualTest extends OpMode {
    public static double GATE_DELAY_S = 0.2;
    public static double FEED_DURATION_S = 0.35;

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, trans, leftflywheel, rightflywheel;
    private ServoImplEx gate, board;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean shooting;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        // 与 ManualOpMode 的方向一致。
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        trans = hardwareMap.get(DcMotorEx.class, "TRANS");
        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        for (DcMotorEx motor : new DcMotorEx[]{intake, trans, leftflywheel, rightflywheel}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        board = hardwareMap.get(ServoImplEx.class, "board");
        Shoot.initBoard(board);
        gate = hardwareMap.get(ServoImplEx.class, "gate");
        gate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(Shoot.GATE_CLOSED_POSITION);
        shooting = false;
        showTelemetry();
    }

    @Override
    public void start() {
        leftflywheel.setPower(1);
        rightflywheel.setPower(1);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftFront.setPower((y + x + rx) / d);
        rightFront.setPower((y - x - rx) / d);
        leftBack.setPower((y - x + rx) / d);
        rightBack.setPower((y + x - rx) / d);

        // 双飞轮以 +1 功率常开。
        leftflywheel.setPower(1);
        rightflywheel.setPower(1);

        if (gamepad1.rightTriggerWasPressed() && !shooting) {
            shooting = true;
            timer.reset();
            gate.setPosition(Shoot.GATE_OPEN_POSITION);
        }

        // 用计时推进发射，底盘每轮仍然更新。
        if (shooting) {
            double elapsed = timer.seconds();
            if (elapsed >= GATE_DELAY_S + FEED_DURATION_S) {
                setFeedPower(0, 0);
                gate.setPosition(Shoot.GATE_CLOSED_POSITION);
                shooting = false;
            } else if (elapsed >= GATE_DELAY_S) {
                setFeedPower(1, 1);
            } else {
                setFeedPower(0, 0);
            }
        } else {
            setFeedPower(gamepad1.left_trigger_pressed ? 1 : 0, 0);
        }

        telemetry.addData("Forward y", "%.2f", y);
        telemetry.addData("Strafe x", "%.2f", x);
        telemetry.addData("Rotate rx", "%.2f", rx);
        telemetry.addData("lf / lb power", "%.2f / %.2f", leftFront.getPower(), leftBack.getPower());
        telemetry.addData("rf / rb power", "%.2f / %.2f", rightFront.getPower(), rightBack.getPower());
        showTelemetry();
    }

    private void setFeedPower(double intakePower, double transPower) {
        intake.setPower(intakePower);
        trans.setPower(transPower);
    }

    private void showTelemetry() {
        telemetry.addLine("有头底盘：左摇杆移动，右摇杆左右转向，松开停止");
        telemetry.addLine("START 后双飞轮常开 +1；L2 按住吸取，R2 点击发射一次");
        telemetry.addData("State", shooting
                ? (timer.seconds() < GATE_DELAY_S ? "GATE DELAY" : "FEEDING") : "IDLE");
        telemetry.addData("Shot elapsed (s)", shooting ? timer.seconds() : 0);
        telemetry.addData("Gate delay / feed duration (s)", "%.2f / %.2f", GATE_DELAY_S, FEED_DURATION_S);
        telemetry.addData("Gate command", gate.getPosition());
        telemetry.addData("Board command", board.getPosition());
        telemetry.addData("INTAKE / TRANS power", "%.1f / %.1f", intake.getPower(), trans.getPower());
        telemetry.addData("Flywheel power (left / right)", "%.1f / %.1f",
                leftflywheel.getPower(), rightflywheel.getPower());
        telemetry.addData("Left velocity (ticks/s)", leftflywheel.getVelocity());
        telemetry.addData("Right velocity (ticks/s)", rightflywheel.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        setFeedPower(0, 0);
        leftflywheel.setPower(0);
        rightflywheel.setPower(0);
        gate.setPosition(Shoot.GATE_CLOSED_POSITION);
        shooting = false;
    }
}
