package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class GameMechanisms {
    private static final double BOARD_HIGH_POSITION = 0.2;
    private static final double BOARD_TRAVEL_SECONDS = 1.0;

    private DcMotor intakeMotor;
    private DcMotor shootMotor;
    private Servo board;
    private Servo flower;
    private final ElapsedTime boardTimer = new ElapsedTime();
    private double boardPosition;
    private double boardStartPosition;
    private boolean boardHeld;
    private boolean boardMovingDown;
    private boolean flowerRaised;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shootMotor = hardwareMap.get(DcMotor.class, "flywheel");
        stop();
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flower = initServos(hardwareMap);
        board = hardwareMap.get(Servo.class, "board");
        boardPosition = BOARD_HIGH_POSITION;
        boardHeld = false;
        boardMovingDown = false;
        flowerRaised = true;
    }

    public static Servo initServos(HardwareMap hardwareMap) {
        hardwareMap.get(Servo.class, "board").setPosition(BOARD_HIGH_POSITION);
        Servo flower = hardwareMap.get(Servo.class, "flower");
        flower.setPosition(1.0);
        return flower;
    }

    public void toggleFlower() {
        flowerRaised = !flowerRaised;
        flower.setPosition(flowerRaised ? 1.0 : 0.0);
    }

    public void updateBoard(boolean held) {
        if (!held) {
            boardHeld = false;
            return;
        }

        if (!boardHeld) {
            boardHeld = true;
            boardMovingDown = !boardMovingDown;
            boardStartPosition = boardPosition;
            boardTimer.reset();
        }

        double movement = BOARD_HIGH_POSITION * boardTimer.seconds() / BOARD_TRAVEL_SECONDS;
        double next = Range.clip(boardStartPosition + (boardMovingDown ? -movement : movement),
                0.0, BOARD_HIGH_POSITION);
        if (next != boardPosition) {
            boardPosition = next;
            board.setPosition(boardPosition);
        }
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
