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
//            hardware.forward(-24)
//                    .spin(-90)
//                    .forward(-12)
//                    .backward(-12+3)
//                    .setIntakePower(0.2)
//                    .sleep(750)
//                    .setIntakePower(0)
//                    .backward(-6);
//            hardware.forward(-3)
//                    .fastSpin(90)
//                    .fastForward(-12)
//                    .fastSpin(0)
//                    .fastForward(-36+3)
//                    .fastBackward(-12-6)
//                    .setIntakePower(-0.6)
//                    .sleep(500)
//                    .setIntakePower(0)
//                    .fastSpin(-90)
//                    .backward(-8)
//                    .setArmPower(0.75)
//                    .sleep(1000)
//                    .setArmPower(0)
//                    .setDumpPosition(0.35)
//                    .sleep(350)
//                    .setArmPower(-0.75)
//                    .sleep(1000)
//                    .setArmPower(0)
//                    .backward(-12)
//                    .setHolderPosition(0)
//                    .sleep(400)
//                    .forward(-4)
//                    .rightShift(-20+6)
//                    .fastBackward(-12);
            hardware.forward(-24)
                    .spin(-90)
                    .forward(-10)
                    .backward(-6)
                    .setIntakePower(0.4)
                    .sleep(750)
                    .setIntakePower(0)
                    .backward(-4);

            //use it when it's finished
//            hardware.sleep(10000)//it's a temporary number,remember to change it!
//                    .fastBackward(-48,2)
//                    .setArmPower(0.75)
//                    .sleep(1200)
//                    .setArmPower(0)
//                    .setDumpPosition(0.4)
//                    .sleep(450)
//                    .setArmPower(-0.75)
//                    .sleep(650)
//                    .setArmPower(0)
//                    .fastBackward(26)    //temporary number
//                    .backward(6)         //temporary number
//                    .sleep(500)
//                    .setHolderPosition(0.5);
            //.forward(2);
        } else if (xCoord < 213) {
            hardware.forward(-24)
                    .spin(90)
                    .forward(-12)
                    .backward(-12+3)
                    .setIntakePower(0.4)
                    .sleep(750)
                    .setIntakePower(0)
                    .backward(-6);


//            hardware.forward(-24)
//                    .spin(90)
//                    .forward(-20)
//                    .backward(-20)
//                    .setIntakePower(0.2)
//                    .sleep(750)
//                    .setIntakePower(0);
//
//            //use it when it's finished
//            hardware.sleep(10000)//it's a temporary number,remember to change it!
//                    .fastBackward(-48,2)
//                    .setArmPower(0.75)
//                    .sleep(1200)
//                    .setArmPower(0)
//                    .setDumpPosition(0.4)
//                    .sleep(450)
//                    .setArmPower(-0.75)
//                    .sleep(650)
//                    .setArmPower(0)
//                    .fastBackward(26)    //temporary number
//                    .backward(6)         //temporary number
//                    .sleep(500)
//                    .setHolderPosition(0.5);
//                    //.forward(2);

            //previous code
//            hardware.forward(-3)
//                    .spin(90)
//                    .forward(-8)
//                    .spin(0)
//                    .forward(-24+3-12)
//                    .backward(-12)
//                    .setIntakePower(0.2)
//                    .sleep(750)
//                    .setIntakePower(0)
//                    .backward(-6);

//            hardware.forward(-3)
//                    .fastSpin(-90)
//                    .fastForward(-12)
//                    .fastSpin(0)
//
//            .fastForward(-24)
//                    .fastSpin(-90)
//                    .fastForward(-12)
//                    .fastBackward(-12)
//                    .setIntakePower(-0.6)
//                    .setArmPower(0.75)
//                    .sleep(1000)
//                    .setIntakePower(0)
//                    .setArmPower(0)
//                    .setDumpPosition(0.35)
//                    .sleep(350)
//                    .setArmPower(-0.75)
//                    .sleep(1000)
//                    .setArmPower(0)
//                    .backward(-12-24)
//                    .setHolderPosition(0)
//                    .sleep(400)
//                    .forward(-4)
//                    .rightShift(-18-6)
//                    .fastBackward(-12);
        } else {
            hardware.forward(-36)
                    .backward(-12+2.5) //it may cause some problem?
                    .setIntakePower(0.4)
                    .sleep(750)
                    .setIntakePower(0)
                    .backward(-4);

//            hardware.sleep(10000)//it's a temporary number,remember to change it!
//                    .spin(90)
//                    .fastBackward(-48,2)
//                    .setArmPower(0.75)
//                    .sleep(1200)
//                    .setArmPower(0)
//                    .setDumpPosition(0.4)
//                    .sleep(450)
//                    .setArmPower(-0.75)
//                    .sleep(650)
//                    .setArmPower(0)
//                    .fastBackward(26)    //temporary number
//                    .backward(6)         //temporary number
//                    .sleep(500)
//                    .setHolderPosition(0.5)
//                    .forward(2);
//                    .backward(-6);
//            hardware.fastForward(-36)
//                    .fastBackward(-12)
//                    .setIntakePower(-0.6)
//                    .sleep(500)
//                    .setIntakePower(0)
//                    .fastSpin(-90)
//                    .fastBackward(-20)
//                    .setArmPower(0.75)
//                    .sleep(1000)
//                    .setArmPower(0)
//                    .setDumpPosition(0.35)
//                    .sleep(350)
//                    .setArmPower(-0.75)
//                    .sleep(1000)
//                    .setArmPower(0)
//                    .backward(-12)
//                    .setHolderPosition(0)
//                    .sleep(400)
//                    .forward(-4)
//                    .rightShift(-18)
//                    .fastBackward(-12);
        }

    }
}
