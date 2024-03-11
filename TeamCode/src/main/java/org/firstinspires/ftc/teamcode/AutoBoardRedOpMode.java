package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class AutoBoardRedOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware hardware = new RobotHardwareImpl(this);
        hardware.initDoubleVision();
        hardware.initMovement();
        hardware.setHolderPosition(0.5);

        waitForStart();

        resetRuntime();

        Recognition recognition = null;
        while (recognition == null && opModeIsActive() && getRuntime() <= 2.0) {
            recognition = hardware.getTfod("RedCube");
            sleep(10);
        }
        double xCoord = recognition == null ? 320 : (recognition.getLeft() + recognition.getRight()) / 2;

        if (xCoord >= 426) {
            hardware.rightShift(-12)
                    .fastForward(-36)
                    .fastBackward(-12-6)
                    .setIntakePower(0.2)
                    .sleep(750)
                    .setIntakePower(0)
                    .spin(90)
                    .backward(-8)
                    .setArmPower(0.75)
                    .sleep(1200)
                    .setArmPower(0)
                    .setDumpPosition(0.35)
                    .sleep(500)
                    .setArmPower(-0.75)
                    .sleep(1000)
                    .setArmPower(0)
                    .fastBackward(-10.5)
                    .driveStraight(0.3, 1, hardware.getHeading())
                    .setHolderPosition(0)
                    .sleep(750)
                    .forward(-6)
                    .leftShift(-20+6)
                    .spin(90)
                    .fastBackward(-18);
        } else if (xCoord < 312) {
            hardware.fastForward(-24)
                    .spin(90)
                    .fastForward(-12)
                    .fastBackward(-12+4)
                    .setIntakePower(0.2)
                    .sleep(750)
                    .setIntakePower(0)
                    .setArmPower(0.75)
                    .sleep(1200)
                    .setArmPower(0)
                    .setDumpPosition(0.35)
                    .sleep(500)
                    .setArmPower(-0.75)
                    .sleep(1000)
                    .setArmPower(0)
                    .fastBackward(-9.5-22)
                    .driveStraight(0.3, 1, hardware.getHeading())
                    .setHolderPosition(0)
                    .sleep(750)
                    .forward(-6)
                    .leftShift(-20-6)
                    .spin(90)
                    .fastBackward(-18);
        } else {
            hardware.fastForward(-36)
                    .fastBackward(-12)
                    .setIntakePower(0.2)
                    .sleep(750)
                    .setIntakePower(0)
                    .spin(90)
                    .fastBackward(-20)
                    .setArmPower(0.75)
                    .sleep(1200)
                    .setArmPower(0)
                    .setDumpPosition(0.35)
                    .sleep(500)
                    .setArmPower(-0.75)
                    .sleep(1000)
                    .setArmPower(0)
                    .fastBackward(-9.5)
                    .driveStraight(0.3, 1, hardware.getHeading())
                    .setHolderPosition(0)
                    .sleep(750)
                    .forward(-6)
                    .leftShift(-24)
                    .spin(90)
                    .fastBackward(-18);
        }

    }
}
