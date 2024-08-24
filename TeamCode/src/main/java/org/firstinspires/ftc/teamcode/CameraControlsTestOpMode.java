package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@TeleOp
public class CameraControlsTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hardware = new RobotHardwareImpl(this);
        hardware.initDoubleVision();

        waitForStart();

        while (hardware.getCameraState() != VisionPortal.CameraState.STREAMING);

        ExposureControl exposureControl = hardware.getCameraControl(ExposureControl.class);
        GainControl gainControl = hardware.getCameraControl(GainControl.class);

        long exposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
        int gain = gainControl.getGain();
        ExposureControl.Mode exposureControlMode = exposureControl.getMode();
        boolean exposureAePriority = exposureControl.getAePriority();

        Gamepad gamepad1Snapshot = copyGamepad(gamepad1);

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !gamepad1Snapshot.dpad_up) {
                exposure += 5;
            }
            if (gamepad1.dpad_down && !gamepad1Snapshot.dpad_down) {
                exposure -= 5;
            }
            if (gamepad1.left_bumper && !gamepad1Snapshot.left_bumper) {
                gain += 5;
            }
            if (gamepad1.right_bumper && !gamepad1Snapshot.right_bumper) {
                gain -= 5;
            }
            if (gamepad1.a && !gamepad1Snapshot.a) {
                switch (exposureControlMode) {
                    case Auto:
                        exposureControlMode = ExposureControl.Mode.Manual;
                        break;
                    case Manual:
                        exposureControlMode = ExposureControl.Mode.Auto;
                        break;
                }
            }
            if (gamepad1.b && !gamepad1Snapshot.b) {
                exposure = exposureControl.getMinExposure(TimeUnit.MILLISECONDS);
                gain = gainControl.getMinGain();
            }
            if (gamepad1.x && !gamepad1Snapshot.x) {
                exposure = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
                gain = gainControl.getMaxGain();
            }
            if (gamepad1.y && !gamepad1Snapshot.y) {
                exposureAePriority = !exposureAePriority;
            }

            exposureControl.setMode(exposureControlMode);
            exposureControl.setAePriority(exposureAePriority);
            if (exposureControlMode == ExposureControl.Mode.Manual) {
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            } else {
                exposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
            }
            gainControl.setGain(gain);

            telemetry.addData("Exposure mode", exposureControlMode.name());
            telemetry.addData("Exposure Ae Priority", exposureAePriority);
            telemetry.addData("Exposure", exposure);
            telemetry.addData("Gain", gain);
        }
    }

    Gamepad copyGamepad(Gamepad source) {
        Gamepad copy = new Gamepad();
        copy.fromByteArray(source.toByteArray());
        return copy;
    }
}
