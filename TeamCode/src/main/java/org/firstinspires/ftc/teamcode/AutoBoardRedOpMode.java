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
        hardware.setHolderPosition(0);
        hardware.setDronePosition(0.94);

        waitForStart();

        resetRuntime();

        Recognition recognition = null;
        while (recognition == null && opModeIsActive() && getRuntime() <= 2.0) {
            recognition = hardware.getTfod("RedCube");
            sleep(10);
        }
        double xCoord = recognition == null ? 320 : (recognition.getLeft() + recognition.getRight()) / 2;

        if (xCoord >= 426) {
            //right
            hardware.fastGotoPosition(19,-10,0)
                    .gotoPosition(13,-10,90)
                    .stretchArm()
                    .gotoPosition2(13,-34.5,90)
                    .sleep(100)
                    .setHolderPosition(0.5)
                    .sleep(400);
        } else if (xCoord < 213) {
            //left
            hardware.fastGotoPosition(20,-3,45)
                    .fastForward(-4)
                    .fastSpin(90)
                    .gotoPosition(19,3.5,90)
                    .fastBackward(-5)
                    .stretchArm()
                    .gotoPosition(24,-34.5,90)
                    .sleep(100)
                    .setHolderPosition(0.5)
                    .sleep(400);
        } else {
            //middle
            hardware.fastGotoPosition(28,-2.8,0)
                    .gotoPosition(18,-2.8,90)
                    .stretchArm()
                    .gotoPosition(18,-34.5,90)
                    .setHolderPosition(0.5)
                    .sleep(400);
        }
        //parking
        hardware.gotoPosition(-6,-29,90)
                .resetArm()
                .gotoPosition2(-6,-44.8,90);

    }
}
