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

        boolean dump;
        boolean on_stretch = false;
        boolean on_recovering = false;
        double start_time = 0;
        double dumpPosition = hardware.getDumpPosition();
        Gamepad gamepad2Snapshot = new Gamepad();
        gamepad2Snapshot.fromByteArray(gamepad2.toByteArray());
        if (hardware.getDumpPosition() > 0.6) {
            dump = false;
        } else {
            dump = true;
        }

        while (opModeIsActive()) {
            double timer = getRuntime();
            if (gamepad2.dpad_up && gamepad2.right_bumper) {
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
                hardware.setHolderPosition(0.5);
            } else {
                hardware.setHolderPosition(0);
            }

            if(!on_recovering && !on_stretch) {
                if (hardware.getDumpPosition() > 0.6) {
                    dump = false;
                } else {
                    dump = true;
                }
            }

            if (gamepad2.y && !gamepad2Snapshot.y && !on_recovering && !on_stretch) {
                dump = !dump;
                if (dump) {
                    start_time = getRuntime() - 0.25;
                    on_stretch = true;
                    dumpPosition = hardware.getDumpPosition();
                }else{
                    start_time = getRuntime() - 0.25;
                    on_recovering = true;
                    dumpPosition = hardware.getDumpPosition();
                }
            }

            if((dump && hardware.getDumpPosition() > 0.6) || (on_stretch && dump)){
                if(timer - start_time >= 0.025 && hardware.getDumpPosition() >= 0.43){
                    dumpPosition -= 0.025;
                    start_time = getRuntime();
                } else if (timer-start_time >= 0.25 && dumpPosition <= 0.43) {
                    dumpPosition = 0.4;
                    on_stretch = false;
                }
                hardware.setDumpPosition(dumpPosition);
            }
            if ((!dump && hardware.getDumpPosition() < 0.6) || (on_recovering && !dump)) {
                if (timer - start_time >= 0.05 && dumpPosition <= 0.95) {
                    dumpPosition += 0.025;
                    start_time = getRuntime();
                }
                else if (timer - start_time >= 0.025 && dumpPosition >= 0.95) {
                    dumpPosition = 0.98;
                    on_recovering = false;
                }
                hardware.setDumpPosition(dumpPosition);
            }
            telemetry.addData("dumpPosition",dumpPosition);

            if (gamepad2.dpad_down) {
                hardware.setDronePosition(0.75);
            } else {
                hardware.setDronePosition(1);
            }

            hardware.driveRobot(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            telemetry.addData("distance",hardware.getDistance());
            gamepad2Snapshot.fromByteArray(gamepad2.toByteArray());
            telemetry.update();
            sleep(10);
        }

        hardware.stopMotor();
    }
}
