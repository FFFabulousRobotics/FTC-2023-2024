package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class


AutoPixelRedOpMode extends LinearOpMode {

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
        double xCoord = 320;
        double yCoord = 400;
        hardware.setHolderPosition(0);
        hardware.setDronePosition(0.94);

        while (yCoord > 240) {

            while (recognition == null && opModeIsActive() && getRuntime() <= 2.0) {
                recognition = hardware.getTfod("RedCube");
                sleep(10);
            }
            xCoord = recognition == null ? 320 : (recognition.getLeft() + recognition.getRight()) / 2;
            yCoord = recognition == null ? 20 : (recognition.getTop() + recognition.getBottom()) / 2;
        }

        if (xCoord >= 426) {
            //right
            hardware.gotoPosition2(22,3,-45)
                    .fastForward(-4)
                    .fastSpin(-90)
                    .fastForward(-4)
                    .fastBackward(-10)
                    .gotoPosition2(25,3,90)
                    .fastForward(-18);
            inhalePixel(hardware);
            hardware.gotoPosition2(47,0,90)
                    .fastBackward(-50)
                    .setArmPower(0.75)
                    .moveDiagonally(16,135)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(450)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .gotoPosition2(15,-80,90)
                    .gotoPosition2(15,-83,90)
                    .setHolderPosition(0.5)
                    .sleep(500);
        } else if (xCoord < 213) {
            //left
            hardware.gotoPosition(24,-3,90)
                    .fastForward(-6)
                    .fastBackward(-6)
                    .gotoPosition2(45.3,21,90)
                    .fastForward(-8);
            inhalePixel(hardware);
            hardware.gotoPosition2(50,0,90)
                    .gotoPosition2(50,-40,90)
                    .setArmPower(0.75)
                    .moveDiagonally(18,135)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(450)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .gotoPosition2(28,-80,90)
                    .gotoPosition2(28,-86,90)
                    .setHolderPosition(0.5)
                    .sleep(500);
        } else {
            //middle
            hardware.gotoPosition(28,0,0)
                    .gotoPosition(25,0,90)
                    .fastForward(-25);
            inhalePixel(hardware);
            hardware.gotoPosition2(55,0,90)
                    .gotoPosition2(55,-50,90)
                    .setArmPower(0.75)
                    .moveDiagonally(18,135)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(450)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .moveDiagonally(10,135)
                    .gotoPosition2(21,-80,90)
                    .gotoPosition2(21,-86,90)
                    .setHolderPosition(0.5)
                    .sleep(500);
        }

    }
    public void inhalePixel(RobotHardware hardware){
        hardware.setIntakePower(0.9)
                .leftShift(-5)
                .rightShift(-10)
                .fastBackward(-4)
                .fastForward(-8)
                .leftShift(-10)
                .rightShift(-5)
                .fastBackward(-4)
                .fastForward(-2.5)
                .backward(-5)
                .sleep(500)
                .setIntakePower(-0.8);
    }
}
