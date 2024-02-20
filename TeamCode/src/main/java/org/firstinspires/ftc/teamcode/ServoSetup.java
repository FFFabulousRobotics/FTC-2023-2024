package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSetup extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "Servo");
    }

    @Override
    public void loop() {
        servo.setPosition(0);
    }
}
