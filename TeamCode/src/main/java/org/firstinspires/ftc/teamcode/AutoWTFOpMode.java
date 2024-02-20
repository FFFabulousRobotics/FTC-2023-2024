package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoWTFOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware(this);

        hardware.initMovement();

        ColorRangeSensor colorSensor = hardwareMap.get(ColorRangeSensor.class, "Color");

        waitForStart();

        hardware.resetYaw();
        hardware.driveStraight(1, 26, 0);


    }
}
