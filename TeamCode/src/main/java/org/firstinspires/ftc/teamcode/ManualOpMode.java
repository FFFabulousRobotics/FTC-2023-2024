package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ManualOpMode extends OpMode {
    MotorController motorController;

    @Override
    public void init() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FL");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FR");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BL");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BR");

        motorController = new MotorController(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            motorController.stop();
        } else {
            motorController.move(
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );
        }
    }

    @Override
    public void stop() {
        motorController.stop();
    }
}
