package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Team2LiftComponent;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

/**
 * Description: Test Team 2's lift component
 * Hardware:
 *  [motor0] Lift Motor (Core Hex)
 *  [motor1] Unused
 *  [motor2] Unused
 *  [motor3] Unused
 *  [servo0] Unused
 *  [servo1] Unused
 *  [servo2] Unused
 *  [servo3] Unused
 * Controls:
 *  [Left Joystick] Pull up or down to lift arm from starting position
 */
@TeleOp(name="Test Set Point Lift", group="Test")
// @Disabled
public class TestSetPointLift extends TeleOpModeBase { // TODO: Test
    private Team2LiftComponent lift;
    private double targetPosition;


    @Override
    public void setup() {
        Motor lift_motor = HardwareMapContainer.motor3;
        lift = new Team2LiftComponent(lift_motor, 0.42, (int)((288 / 3) / (Math.PI*2)), 0); // Core Hex Motor has 288 counts/revolution; counts/radian = counts/revn / (radians/revn); 3:1 gear
        new GamepadButton(Inputs.gamepad1, PSButtons.SQUARE).whenPressed(() -> {
            setHeight(0);
        });
        new GamepadButton(Inputs.gamepad1, PSButtons.TRIANGLE).whenPressed(() -> {
            setHeight(0.5);
        });
        new GamepadButton(Inputs.gamepad1, PSButtons.CROSS).whenPressed(() -> {
            setHeight(1);
        });
    }

    private void setHeight(double targetPos) {
        this.targetPosition = targetPos;
        try {
            lift.setHeight(this.targetPosition);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void every_tick() {
//        if (Math.abs(Inputs.gamepad1.getLeftY()) != this.targetPosition) {
//            this.targetPosition = Math.abs(Inputs.gamepad1.getLeftY());
//        } else {
//            telemetry.addLine("[Lift] BUSY");
//        }

//        telemetry.addData("[Lift] Position of gamepad", Math.abs(Inputs.gamepad1.getLeftY()));

        telemetry.addData("[Lift] Last set position to", this.targetPosition);
    }
}
