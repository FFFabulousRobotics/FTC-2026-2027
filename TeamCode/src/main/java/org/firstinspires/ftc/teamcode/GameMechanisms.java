package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GameMechanisms {
    private DcMotor intakeMotor;
    private DcMotor shootMotor;
    private Servo board;
    private Servo flower;
    private boolean boardAtLowPosition;
    private boolean flowerRaised;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shootMotor = hardwareMap.get(DcMotor.class, "flywheel");
        stop();
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flower = initServos(hardwareMap);
        board = hardwareMap.get(Servo.class, "board");
        boardAtLowPosition = false;
        flowerRaised = true;
    }

    public static Servo initServos(HardwareMap hardwareMap) {
        hardwareMap.get(Servo.class, "board").setPosition(0.2);
        Servo flower = hardwareMap.get(Servo.class, "flower");
        flower.setPosition(1.0);
        return flower;
    }

    public void toggleFlower() {
        flowerRaised = !flowerRaised;
        flower.setPosition(flowerRaised ? 1.0 : 0.0);
    }

    public void toggleBoard() {
        boardAtLowPosition = !boardAtLowPosition;
        board.setPosition(boardAtLowPosition ? 0.1 : 0.2);
    }

    public void intake() {
        intakeMotor.setPower(1.0);
        shootMotor.setPower(0.0);
    }

    public void shoot() {
        intakeMotor.setPower(1.0);
        shootMotor.setPower(1.0);
    }

    public void stop() {
        intakeMotor.setPower(0.0);
        shootMotor.setPower(0.0);
    }
}
