package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ManualOpMode extends LinearOpMode {
    RobotHardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardwareImpl(this);

        hardware.initMovement();

        waitForStart();

        boolean dump = false;
        Gamepad gamepad1Snapshot = new Gamepad();
        gamepad1Snapshot.fromByteArray(gamepad1.toByteArray());

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && gamepad1.right_bumper) {
                hardware.setLiftPower(1);
            } else if (gamepad1.dpad_up && !gamepad1.right_bumper && hardware.getLiftPosition() < 2600) {
                hardware.setLiftPower(-1);
            } else {
                hardware.setLiftPower(0);
            }

            if (gamepad1.b && gamepad1.right_bumper) {
                hardware.setArmPower(-0.5);
            } else if (gamepad1.b && !gamepad1.right_bumper) {
                hardware.setArmPower(0.5);
            } else {
                hardware.setArmPower(0);
            }

            if (gamepad1.x && gamepad1.right_bumper) {
                hardware.setIntakePower(-0.9);
            } else if (gamepad1.x && !gamepad1.right_bumper) {
                hardware.setIntakePower(0.9);
            } else {
                hardware.setIntakePower(0);
            }

            if (gamepad1.a) {
                hardware.setHolderPosition(0);
            } else {
                hardware.setHolderPosition(0.5);
            }

            if (gamepad1.y && !gamepad1Snapshot.y) {
                dump = !dump;
            }

            if (dump && hardware.getDumpPosition() > 0.6) {
                hardware.setDumpPosition(0.35);
            } else if (!dump && hardware.getDumpPosition() < 0.6){
                for (double dumpPosition = 0.45; dumpPosition <= 0.95; dumpPosition += 0.025) {
                    hardware.setDumpPosition(dumpPosition);
                    sleep(75);
                }
                hardware.setDumpPosition(0.98);
            }

            if (gamepad1.dpad_down) {
                hardware.setDronePosition(0);
            } else {
                hardware.setDronePosition(1);
            }

            hardware.driveRobot(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            gamepad1Snapshot.fromByteArray(gamepad1.toByteArray());
            telemetry.update();
            sleep(10);
        }

        hardware.stopMotor();
    }
}
