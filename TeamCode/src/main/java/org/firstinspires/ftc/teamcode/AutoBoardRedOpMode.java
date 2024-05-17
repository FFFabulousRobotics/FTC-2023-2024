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
            hardware.forward(-3)
                    .spin(-90)
                    .forward(-10)
                    .spin(0)
                    .fastForward(-36+3)
                    .fastBackward(-12-3)
                    .setIntakePower(0.4)
                    .sleep(500)
                    .setIntakePower(0)
                    .gotoDistance(27,-4)
                    //.fastBackward(-4)
                    .spin(90)
                    .fastBackward(-8)
                    .setArmPower(0.75)
                    .sleep(1000)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(350)
                    .setArmPower(-0.75)
                    .sleep(750)
                    .setArmPower(0)
                    .gotoDistance(7.6,-10.5)
                    //.fastBackward(-11)
                    .setHolderPosition(0)
                    .sleep(400)
                    .forward(-4)
                    .leftShift(-17)
                    .backward(-10);
        } else if (xCoord < 312) {
            hardware.fastForward(-24)
                    .spin(90)
                    .forward(-12)
                    .fastBackward(-12)
                    .setIntakePower(0.4)
                    .setArmPower(0.75)
                    .sleep(1000)
                    .setIntakePower(0)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(350)
                    .setArmPower(-0.75)
                    .sleep(750)
                    .setArmPower(0)
                    .fastBackward(-16)
                    .rightShift(-2)
                    .fastBackward(-15.5)
                    .setHolderPosition(0)
                    .sleep(400)
                    .fastForward(-7)
                    .leftShift(-31)
                    .fastBackward(-15);
        } else {
            hardware.fastForward(-36)
                    .fastBackward(-8)
                    .setIntakePower(0.4)
                    .sleep(500)
                    .setIntakePower(0)
                    .gotoDistance(31,-3)
                    //.backward(-8)
                    .spin(90)
                    .fastBackward(-20,3)
                    .setArmPower(0.75)
                    .sleep(1000)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(350)
                    .setArmPower(-0.65)
                    .sleep(750)
                    .setArmPower(0)
                    .gotoDistance(7.6,-11.5)
                    //.backward(-10.5)
                    .setHolderPosition(0)
                    .sleep(400)
                    .forward(-4)
                    .leftShift(-22)
                    .fastBackward(-10);
        }

    }
}
