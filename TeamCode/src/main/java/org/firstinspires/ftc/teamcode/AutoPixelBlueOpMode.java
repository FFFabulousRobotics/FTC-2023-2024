package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class AutoPixelBlueOpMode extends LinearOpMode {

    static int STRETCH_TIME = 1000;
    static int DUMP_TIME = 400;
    static int ARM_BACK = 650;
    @Override
    public void runOpMode() {

        RobotHardware hardware = new RobotHardwareImpl(this);
        hardware.initDoubleVision();
        hardware.initMovement();
        hardware.setHolderPosition(0);
        hardware.setDronePosition(0.94);

        waitForStart();

        resetRuntime();
        hardware.resetYaw();

        Recognition recognition = null;
        while (recognition == null && opModeIsActive() && getRuntime() <= 2.0) {
            recognition = hardware.getTfod("BlueCube");
            sleep(10);
        }
        double xCoord = recognition == null ? 320 : (recognition.getLeft() + recognition.getRight()) / 2;

        if (xCoord < 213) {
            //left
            hardware.gotoPosition(24,-3,90)
                    .fastForward(-6)
                    .gotoPosition(22,-7,-90)
                    .fastForward(-21);
            inhalePixel(hardware);
            hardware.gotoPosition2(55,0,-90)
                    .gotoPosition2(55,50,-90)
                    .setArmPower(0.75)
                    .moveDiagonally(18,-135)
                    .setArmPower(0)
                    .setDumpPosition(0.26)
                    .sleep(500)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .gotoPosition2(20,73,-90)
                    .sleep(100)
                    .setHolderPosition(0.5)
                    .sleep(500);
        } else if (xCoord >= 426) {
            //right
            hardware.gotoPosition(24,3,-45)
                    .fastForward(-4)
                    .fastSpin(-90)
                    .fastForward(-3)
                    .fastBackward(-10)
                    .gotoPosition2(47,-22,-90)
                    .fastForward(-8);
            inhalePixel(hardware);
            hardware.gotoPosition2(55,0,-90)
                    .gotoPosition2(55,40,-90)
                    .setArmPower(0.75)
                    .moveDiagonally(18,-135)
                    .setArmPower(0)
                    .setDumpPosition(0.26)
                    .sleep(500)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .moveDiagonally(10,-135)
                    .gotoPosition2(33.5,73,-90)
                    .setHolderPosition(0.5)
                    .sleep(500);

        } else {
            //middle
            hardware.gotoPosition(28,0,0)
                    .gotoPosition(24,0,-90)
                    .fastForward(-25);
            inhalePixel(hardware);
            hardware.gotoPosition2(55,0,-90)
                    .gotoPosition2(55,40,-90)
                    .setArmPower(0.75)
                    .moveDiagonally(18,-135)
                    .setArmPower(0)
                    .setDumpPosition(0.26)
                    .sleep(500)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .gotoPosition2(28,73,-90)
                    .gotoPosition(28,73,-90)
                    .setHolderPosition(0.5)
                    .sleep(500);
        }

    }

    public void inhalePixel(RobotHardware hardware){
        hardware.setIntakePower(0.9)
                .sleep(500)
                .fastBackward(-5)
                .sleep(500)
                .fastForward(-5)
                .sleep(500)
                .fastBackward(-8)
                .sleep(1000)
                .setIntakePower(-0.8);
    }

}
