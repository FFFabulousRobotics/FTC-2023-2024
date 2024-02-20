package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.inspection.GamepadInspection;

@TeleOp
public class ManualOpMode extends LinearOpMode {
    RobotHardware hardware;

    boolean absoluteMovement = true;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware(this);
        boolean preY = false;

        hardware.initMovement();

//        ColorRangeSensor crs = hardwareMap.get(ColorRangeSensor.class, "Color");

        waitForStart();

        Gamepad gamepad1Snapshot = new Gamepad();
        gamepad1Snapshot.fromByteArray(gamepad1.toByteArray());

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && gamepad1.right_bumper) {
                hardware.setLiftPower(-0.5);
            } else if (gamepad1.dpad_up && !gamepad1.right_bumper && hardware.getLiftPosition() < 2600) {
                hardware.setLiftPower(0.5);
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
                hardware.setHolderPosition(0.4);
            } else {
                hardware.setHolderPosition(0);
            }
//            if (gamepad1.y && !preY) {
//                hardware.setDumpState(true)
//                        .setDumpPosition(1)
//                        .sleep(100)
//                        .setDumpState(false);
//            } else if (!gamepad1.y && preY) {
//                hardware.setDumpState(true)
//                        .setDumpPosition(0)
//                        .sleep(200)
//                        .setDumpState(false);
//            } else (gamepad1.y) {
//                hardware.setDumpPosition(1);
//            }

            if (gamepad1.y) {
                hardware.setDumpPosition(1);
            } else {
                hardware.setDumpPosition(0);
            }

            if (gamepad1.left_bumper) {
                hardware.setDumpState(true);
            } else {
                hardware.setDumpState(false);
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
            telemetry.addData("Lift", hardware.getLiftPosition());
//            telemetry.addData("Distance", crs.getDistance(DistanceUnit.CM) + " cm");
            telemetry.update();
            preY = gamepad1.y;
            sleep(10);
        }

        hardware.stopMotor();
    }
}
