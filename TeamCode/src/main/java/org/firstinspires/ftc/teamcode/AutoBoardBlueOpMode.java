package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class AutoBoardBlueOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware hardware = new RobotHardwareImpl(this);
        hardware.initDoubleVision();
        hardware.initMovement();
        hardware.setHolderPosition(0.5);

        waitForStart();

        resetRuntime();
        hardware.resetYaw();

        Recognition recognition = null;
        while (recognition == null && opModeIsActive() && getRuntime() <= 2.0) {
            recognition = hardware.getTfod("BlueCube");
            sleep(10);
        }
        double xCoord = recognition == null ? 320 : (recognition.getLeft() + recognition.getRight()) / 2;

        //!!!!!!!!
        //                              xCoord = 500;
        //This line is used to debug,delete it when it's finished!
        //!!!!!!!!


        //left 27;mid 31;right 41
        //distance 7.5-7.7
        if (xCoord < 213) {
            hardware.forward(-3)
                    .spin(90)
                    .forward(-10)
                    .spin(0)
                    .fastForward(-36+3)
                    .fastBackward(-12-3)
                    .setIntakePower(0.4)
                    .sleep(500)
                    .setIntakePower(0)
                    .gotoDistance(27,-4)
                    //.fastBackward(-4)
                    .spin(-90)
                    .fastBackward(-8)
                    .setArmPower(0.75)
                    .sleep(1000)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(350)
                    .setArmPower(-0.75)
                    .sleep(850)
                    .setArmPower(0)
                    .gotoDistance(7.6,-11.5)
                    //.fastBackward(-11)
                    .setHolderPosition(0)
                    .sleep(400)
                    .forward(-4)
                    .rightShift(-17)
                    .backward(-10);
        } else if (xCoord >= 426) {
            hardware.fastForward(-24)
                    .spin(-90)
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
                    .sleep(850)
                    .setArmPower(0)
                    .fastBackward(-16)
                    .leftShift(-2)
                    .fastBackward(-15.5)
                    .setHolderPosition(0)
                    .sleep(400)
                    .fastForward(-7)
                    .rightShift(-31)
                    .fastBackward(-15);
        } else {
            hardware.fastForward(-36)
                    .fastBackward(-8)
                    .setIntakePower(0.4)
                    .sleep(500)
                    .setIntakePower(0)
                    .gotoDistance(31,-3)
                    //.backward(-8)
                    .spin(-90)
                    .fastBackward(-20,3)
                    .setArmPower(0.75)
                    .sleep(1200)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(350)
                    .setArmPower(-0.65)
                    .sleep(850)
                    .setArmPower(0)
                    .gotoDistance(7.6,-10.5)
                    //.backward(-10.5)
                    .setHolderPosition(0)
                    .sleep(400)
                    .forward(-4)
                    .rightShift(-22)
                    .fastBackward(-10);
        }

    }
}
