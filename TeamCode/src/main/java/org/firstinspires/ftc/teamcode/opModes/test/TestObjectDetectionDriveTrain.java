package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

import java.util.ArrayList;

@Disabled
@TeleOp(name="Test Object Detection Drive Train [sbottingota]", group="Test")
public class TestObjectDetectionDriveTrain extends TeleOpModeBase {
    Motor leftMotor;
    Motor rightMotor;



    //HardwareMap hardwar;

    //ArrayList<Integer> distances = new ArrayList<Integer>();
    SensorDistance sensorDistance;

    @Override
    public void setup() {
        sensorDistance = new SensorRevTOFDistance(HardwareMapContainer.getMap(), "sensorDistance");

        leftMotor = HardwareMapContainer.motor0;
        rightMotor = HardwareMapContainer.motor1;
    }

    @Override
    public void every_tick() {
        final double detectionDistance = 30D; //in CM

        arcadeDrive(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());

        while (sensorDistance.getDistance(DistanceUnit.CM) <= detectionDistance) {
            arcadeDrive(-Inputs.gamepad1.getLeftX(), -Inputs.gamepad1.getLeftY());
        }
    }

    private void arcadeDrive(double x, double y) {
        leftMotor.set((x - y) / 2D);
        rightMotor.set((x + y) / 2D);
    }
}
