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

//        !!!!!!!!
//        xCoord = 500;
//        This line is used to debug,delete it when it's finished!
//        !!!!!!!!
        if (xCoord < 213) {
            //left
            hardware.fastGotoPosition(22,10.5,0)
                    .gotoPosition(17.6,10.5,-90)
                    .stretchArm()
                    .gotoPosition2(20.6,30,-90)
                    .sleep(100)
                    .setHolderPosition(0.5)
                    .sleep(400);
        } else if (xCoord >= 426) {
            //right
            hardware.fastGotoPosition(20,3,-45)
                    .fastForward(-4)
                    .fastSpin(-90)
                    .gotoPosition(24,-8.5,-90)
                    .fastBackward(-5)
                    .stretchArm()
                    .gotoPosition(33,30,-90)
                    .sleep(100)
                    .setHolderPosition(0.5)
                    .sleep(400);
        } else {
            //middle
            hardware.fastGotoPosition(28,2.8,0)
                    .gotoPosition(20.8,2.8,-90)
                    .stretchArm()
                    .gotoPosition(25,28.5,-90)
                    .setHolderPosition(0.5)
                    .sleep(400);
        }
        //parking
        hardware.gotoPosition(2,21,-90)
                .resetArm()
                .gotoPosition2(2,35.8,-90);

    }
}
