package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

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
        while (hardware.getCameraState() != VisionPortal.CameraState.STREAMING) ;
        hardware.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        hardware.getCameraControl(ExposureControl.class).setExposure(80, TimeUnit.MILLISECONDS);
        hardware.getCameraControl(GainControl.class).setGain(0);

        Recognition recognition = null;
        while (recognition == null && opModeIsActive() && getRuntime() <= 2.0) {
            recognition = hardware.getTfod("RedCube");
            sleep(10);
        }
        double xCoord = recognition == null ? 320 : (recognition.getLeft() + recognition.getRight()) / 2;

        if (xCoord >= 426) {
            //right
            hardware.fastGotoPosition(23, -12, 0)
                    .gotoPosition(15, -12, 90)
                    .stretchArm()
                    .gotoPosition2(15, -36, 90)
                    .sleep(100)
                    .setHolderPosition(0.5)
                    .sleep(400);
        } else if (xCoord < 213) {
            //left
            hardware.fastGotoPosition(16, -3, 45)
                    .fastForward(-4)
                    .fastSpin(90)
                    .gotoPosition(22, 3.5, 90)
                    .fastBackward(-5)
                    .stretchArm()
                    .gotoPosition(28, -36, 90)
                    .sleep(100)
                    .setHolderPosition(0.5)
                    .sleep(400);
        } else {
            //middle
            hardware.fastGotoPosition(25, -2.8, 0)
                    .gotoPosition(22, -2.8, 90)
                    .stretchArm()
                    .gotoPosition(20, -36, 90)
                    .setHolderPosition(0.5)
                    .sleep(400);
        }
        //parking
        hardware.gotoPosition(-6, -29, 90)
                .resetArm()
                .gotoPosition2(-6, -44.8, 90);

    }
}
