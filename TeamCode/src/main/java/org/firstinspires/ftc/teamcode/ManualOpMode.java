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
        double start_time = 0;
        double dumpPosition = 0;
        Gamepad gamepad2Snapshot = new Gamepad();
        gamepad2Snapshot.fromByteArray(gamepad2.toByteArray());

        while (opModeIsActive()) {
            if (gamepad2.dpad_up && gamepad2.right_bumper) {
            double timer = getRuntime();
            if (gamepad1.dpad_up && gamepad1.right_bumper) {
                hardware.setLiftPower(1);
            } else if (gamepad2.dpad_up && !gamepad2.right_bumper && hardware.getLiftPosition() < 2600) {
                hardware.setLiftPower(-1);
            } else {
                hardware.setLiftPower(0);
            }

            if (gamepad2.b && gamepad2.right_bumper) {
                hardware.setArmPower(-0.5);
            } else if (gamepad2.b && !gamepad2.right_bumper) {
                hardware.setArmPower(0.5);
            } else {
                hardware.setArmPower(0);
            }

            if (gamepad2.x && gamepad2.right_bumper) {
                hardware.setIntakePower(-0.9);
            } else if (gamepad2.x && !gamepad2.right_bumper) {
                hardware.setIntakePower(0.9);
            } else {
                hardware.setIntakePower(0);
            }

            if (gamepad2.a) {
                hardware.setHolderPosition(0);
            } else {
                hardware.setHolderPosition(0.5);
            }

            if (gamepad2.y && !gamepad2Snapshot.y) {
                dump = !dump;
                if(!dump) {
                    start_time = getRuntime();
                    dumpPosition = hardware.getDumpPosition();
                }
            }

            if (dump && hardware.getDumpPosition() > 0.6) {
                hardware.setDumpPosition(0.35);
            } else if (!dump && hardware.getDumpPosition() < 0.6){
                if (timer-start_time >= 0.075 && dumpPosition <=0.95) {
                    hardware.setDumpPosition(dumpPosition);
                    dumpPosition += 0.025;
                    start_time = getRuntime();
                } else if (dumpPosition >= 0.95) {
                    hardware.setDumpPosition(0.98);
                }
            }

            if (gamepad2.dpad_down) {
                hardware.setDronePosition(0);
            } else {
                hardware.setDronePosition(1);
            }

            hardware.driveRobot(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            gamepad2Snapshot.fromByteArray(gamepad2.toByteArray());
            telemetry.update();
            sleep(10);
        }

        hardware.stopMotor();
    }
}
