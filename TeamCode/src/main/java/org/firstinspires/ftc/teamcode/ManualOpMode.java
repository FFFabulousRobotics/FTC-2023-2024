package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ManualOpMode extends LinearOpMode {
    RobotHardware hardware;

    static double DUMP_JUDGING_THRESHOLD = 0.75;
    static double LIFT_EXTRACT_POWER = 1;
    static double LIFT_RETRACT_POWER = -1;
    static double ARM_EXTRACT_POWER = -0.5;
    static double ARM_RETRACT_POWER = 0.5;
    static double INTAKE_ROLL_IN_POWER = -0.8;
    static double INTAKE_ROLL_OUT_POWER = 0.8;
    static double HOLDER_OPEN_POSITION = 0.5;
    static double HOLDER_CLOSE_POSITION = 0.2;
    static double HOLDER_FIXING_POSITION = 0;
    static double DRONE_OPEN_POSITION = 0.3;
    static double DRONE_CLOSE_POSITION = 0.7;
    static double CYCLE_TIME_SECONDS = 0.04;
    static double DUMP_STRETCHING_FAST_THRESHOLD = 0.3;
    static double DUMP_RETRACTING_FAST_THRESHOLD = 0.8;
    static double DUMP_STRETCHED_POSITION = 0.26;
    static double DUMP_RETRACTED_POSITION = 0.84;
    static double DUMP_CHANGING_OFFSET = 0.025;


    @Override
    public void runOpMode() {
        hardware = new RobotHardwareImpl(this);

        hardware.initMovement();

        waitForStart();

        DumpState dumpState;
        double checkpoint = 0;
        double targetDumpPosition = hardware.getDumpPosition();
//        Gamepad gamepad2Snapshot = new Gamepad();
//        gamepad2Snapshot.fromByteArray(gamepad2.toByteArray());
        if (hardware.getDumpPosition() < DUMP_JUDGING_THRESHOLD && hardware.getDumpPosition() != 0) {
            dumpState = DumpState.STRETCHED;
        } else {
            dumpState = DumpState.RETRACTED;
        }

        while (opModeIsActive()) {
            double timer = getRuntime();
            if (gamepad2.dpad_up && gamepad2.right_bumper) {
                hardware.setLiftPower(LIFT_EXTRACT_POWER);
            } else if (gamepad2.dpad_up && !gamepad2.right_bumper && hardware.getLiftPosition() < 2600) {
                hardware.setLiftPower(LIFT_RETRACT_POWER);
            } else {
                hardware.setLiftPower(0);
            }

            if (gamepad2.b && gamepad2.right_bumper) {
                hardware.setArmPower(ARM_EXTRACT_POWER);
            } else if (gamepad2.b && !gamepad2.right_bumper) {
                hardware.setArmPower(ARM_RETRACT_POWER);
            } else {
                hardware.setArmPower(0);
            }

            if (gamepad2.x && gamepad2.right_bumper) {
                hardware.setIntakePower(INTAKE_ROLL_IN_POWER);
            } else if (gamepad2.x && !gamepad2.right_bumper) {
                hardware.setIntakePower(INTAKE_ROLL_OUT_POWER);
            } else {
                hardware.setIntakePower(0);
            }

            if (gamepad2.a) {
                hardware.setHolderPosition(HOLDER_OPEN_POSITION);
            } else if (gamepad2.left_bumper) {
                hardware.setHolderPosition(HOLDER_FIXING_POSITION);
            } else{
                hardware.setHolderPosition(HOLDER_CLOSE_POSITION);
            }

            if (gamepad2.y && dumpState.isAtIdle()) {
                checkpoint = timer;
                switch (dumpState) {
                    case RETRACTED:
                        dumpState = DumpState.STRETCHING;
                        break;
                    case STRETCHED:
                        dumpState = DumpState.RETRACTING;
                        break;
                }
                if(hardware.getDumpPosition() == 0){hardware.setDumpPosition(DUMP_RETRACTED_POSITION);}
                targetDumpPosition = hardware.getDumpPosition();
            }


            if (timer - checkpoint >= CYCLE_TIME_SECONDS) {
                switch (dumpState) {
                    case STRETCHING:
                        if (hardware.getDumpPosition() >= DUMP_STRETCHING_FAST_THRESHOLD) {
                            targetDumpPosition -= DUMP_CHANGING_OFFSET;
                            checkpoint = timer;
                        } else if (hardware.getDumpPosition() < DUMP_STRETCHING_FAST_THRESHOLD) {
                            targetDumpPosition = DUMP_STRETCHED_POSITION;
                            dumpState = DumpState.STRETCHED;
                        }
                        hardware.setDumpPosition(targetDumpPosition);
                        break;
                    case RETRACTING:
                        if (hardware.getDumpPosition() <= DUMP_RETRACTING_FAST_THRESHOLD) {
                            targetDumpPosition += DUMP_CHANGING_OFFSET;
                            checkpoint = timer;
                        } else if (hardware.getDumpPosition() >= DUMP_RETRACTING_FAST_THRESHOLD) {
                            targetDumpPosition = DUMP_RETRACTED_POSITION;
                            dumpState = DumpState.RETRACTED;
                        }
                        hardware.setDumpPosition(targetDumpPosition);
                        break;
                }
            }


            telemetry.addData("targetDumpPosition", targetDumpPosition);
            telemetry.addData("dumpPosition",hardware.getDumpPosition());
            telemetry.addData("dumpstate",dumpState);

            if (gamepad2.dpad_down) {
                hardware.setDronePosition(DRONE_OPEN_POSITION);
            } else {
                hardware.setDronePosition(DRONE_CLOSE_POSITION);
            }

            hardware.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("distance", hardware.getDistance());
//            gamepad2Snapshot.fromByteArray(gamepad2.toByteArray());
            telemetry.update();
            sleep(10);
        }

        hardware.stopMotor();
    }

    enum DumpState {
        RETRACTED, STRETCHED, RETRACTING, STRETCHING;

        public boolean isAtIdle() {
            return this == RETRACTED || this == STRETCHED;
        }
    }
}
