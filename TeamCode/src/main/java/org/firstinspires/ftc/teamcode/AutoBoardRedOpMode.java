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
            hardware.forward(-3)
                    .spin(-90)
                    .forward(-10)
                    .spin(0)
                    .fastForward(-36+3)
                    .fastBackward(-16)
                    .setIntakePower(0.4)
                    .sleep(500)
                    .setIntakePower(0)
                    .gotoDistance(27,-4)
                    //.fastBackward(-4)
                    .spin(90)
                    .fastBackward(-8)
                    .setArmPower(0.75)
                    .sleep(1200)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(450)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .fastBackward(-12)
                    //.fastBackward(-11)
                    .sleep(500)
                    .setHolderPosition(0.5);
//                    .sleep(400)
//                    .forward(-4)
//                    .leftShift(-19)
//                    .backward(-10);
        } else if (xCoord < 213) {
            hardware.fastForward(-24)
                    .spin(90)
                    .forward(-12)
                    .fastBackward(-7)
                    .setIntakePower(0.4)
                    .setArmPower(0.75)
                    .sleep(600)
                    .setIntakePower(0)
                    .sleep(400)
                    .setArmPower(0)
                    .setDumpPosition(0.4)
                    .sleep(350)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .fastBackward(-18)
                    .rightShift(-2)
                    .fastBackward(-14,2)
                    .setHolderPosition(0.5);
//                    .sleep(400)
//                    .fastForward(-7)
//                    .leftShift(-31)
//                    .fastBackward(-15);
        } else {
            hardware.fastForward(-36)
                    .fastBackward(-9)
                    .setIntakePower(0.3)
                    .sleep(750)
                    .setIntakePower(0)
                    .gotoDistance(31,-3)
                    .spin(90)
                    .fastBackward(-20)
                    .setArmPower(0.75)
                    .sleep(1000)
                    .setArmPower(0)
                    .setDumpPosition(0.35)
                    .sleep(500)
                    .setArmPower(-0.75)
                    .sleep(650)
                    .setArmPower(0)
                    .fastBackward(-10.5)
                    .driveStraight(0.3, 1, hardware.getHeading())
                    .setHolderPosition(0.5);
//                    .sleep(750)
//                    .forward(-6)
//                    .leftShift(-24)
//                    .spin(90)
//                    .fastBackward(-18);
        }

    }
}
