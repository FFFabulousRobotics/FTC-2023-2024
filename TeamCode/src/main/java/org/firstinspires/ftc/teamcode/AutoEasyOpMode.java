package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(preselectTeleOp = "ManualOpMode")
public class AutoEasyOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hardware = new RobotHardware(this);
        hardware.initMovement();
        waitForStart();
        hardware.resetYaw();
        hardware
                .driveStraight(1, -12, 0)
                .turnToHeading(1, -90)
                .driveStrafe(1, -12, -90)
                .driveStraight(1, -24-18, -90)
                .setIntakePower(-0.8)
        ;
        sleep(2000);
        hardware.setIntakePower(0);
        
        while (opModeIsActive());
    }
}
