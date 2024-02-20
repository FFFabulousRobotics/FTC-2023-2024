package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import java.util.Map;

@SuppressWarnings("unused")
public class HardwareConfig {
    public Hardware hardware;

    public static class Hardware {
        public ChassisConfig chassis;
        public GyroConfig gyro;
        public MotorsConfig motors;
        public ServosConfig servos;
    }

    public static class ChassisConfig {
        public ChassisType chassisType;
        public Object motors;
        public Map<String, AnalogInput> control;
    }

    public enum ChassisType {
        OMNI, TANK
    }

    public static class Motor {
        public String id;
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
        public DcMotor.Direction direction;
        public DigitalInput keyBind;
        public double speed;
    }

    public enum AnalogInput {
        left_stick_x,
        left_stick_y,
        left_trigger,
        right_stick_x,
        right_stick_y,
        right_trigger,
        touchpad_finger_1_x,
        touchpad_finger_1_y,
        touchpad_finger_2_x,
        touchpad_finger_2_y
    }

    public enum DigitalInput {
        a,
        b,
        back,
        circle,
        cross,
        dpad_down,
        dpad_left,
        dpad_right,
        dpad_up,
        guide,
        left_bumper,
        left_stick_button,
        options,
        ps,
        right_bumper,
        right_stick_button,
        share,
        square,
        start,
        touchpad,
        touchpad_finger_1,
        touchpad_finger_2,
        triangle,
        x,
        y
    }

    public static class GyroConfig {
        public HubOrientation hubOrientation;
        public String imuName;
    }

    public static class HubOrientation {
        public RevHubOrientationOnRobot.LogoFacingDirection logo;
        public RevHubOrientationOnRobot.UsbFacingDirection usb;
    }

    public static class MotorsConfig {
        public List<Motor> motors;
    }

    public static class ServosConfig {
        public List<Servo> servos;
    }

    public static class Servo {
        public String id;
        public DigitalInput keyBind;
        public ServoAngleRestrictions angleRestrictions;
    }

    public static class ServoAngleRestrictions {
        public double min;
        public double max;
    }
}
