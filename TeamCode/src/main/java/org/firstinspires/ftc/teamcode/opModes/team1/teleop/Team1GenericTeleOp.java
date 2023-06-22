package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.components.HDriveWrapper;
import org.firstinspires.ftc.teamcode.components.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.LiftComponent;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * This isn't an OpMode - create an instance of this in each team-specific OpMode
 */
@Config
public class Team1GenericTeleOp {
    public static final GamepadKeys.Button TOGGLE_TURN_MODE_BUTTON = GamepadKeys.Button.LEFT_STICK_BUTTON;
    public static final GamepadKeys.Button RESET_IMU_BUTTON = GamepadKeys.Button.RIGHT_STICK_BUTTON;
    public static final GamepadKeys.Button CLOSE_GRABBER_BUTTON = GamepadKeys.Button.LEFT_BUMPER;
    public static final GamepadKeys.Button OPEN_GRABBER_BUTTON = GamepadKeys.Button.RIGHT_BUMPER;
    public static final GamepadKeys.Button SET_LIFT_TO_GROUND_BUTTON = GamepadKeys.Button.A; // Cross
    public static final GamepadKeys.Button SET_LIFT_TO_LOW_BUTTON = GamepadKeys.Button.B; // Circle
    public static final GamepadKeys.Button SET_LIFT_TO_HIGH_BUTTON = GamepadKeys.Button.Y; // Circle
    public static final GamepadKeys.Button SET_LIFT_TO_MIDDLE_BUTTON = GamepadKeys.Button.X; // Circle

    TeamColour teamColour;
    IMU imu;
    HDriveWrapper drive;
    LiftComponent lift;

    GrabberComponent grabber;

    // If pushed down, the robot can freely turn; the turn is relative to the current direction
    // Otherwise, snap to the nearest cardinal direction; the turn is absolute
    GamepadButton turnModeSwitch;
    public static double RELATIVE_TURN_SPEED_MULTIPLIER = 0.3;

    public void setup(TeamColour teamColor){
        this.grabber = new GrabberComponent(0, 1);
        this.teamColour = teamColor;
        imu = HardwareMapContainer.getMap().get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Get the third motor as a spinner motor
        drive = new HDriveWrapper(new HDrive(
                HardwareMapContainer.motor0,
                HardwareMapContainer.motor1,
                HardwareMapContainer.motor3,
                0,
                Math.PI,
                Math.PI/2
        ), imu);
        lift = new LiftComponent(HardwareMapContainer.motor2, LiftComponent.LiftPosition.GROUND);

        // Reset IMU heading on button press
        // This can also be activated by the button on the back of the joystick
        // That makes the robot consider the direction it was facing the front (pointing to the right from driver's point of view)
        Inputs.gamepad1.getGamepadButton(RESET_IMU_BUTTON).whenPressed(new InstantCommand(() -> {
            imu.resetYaw();
        }));
        imu.resetYaw(); // At the start so no sudden turn

        Inputs.gamepad1.getGamepadButton(CLOSE_GRABBER_BUTTON).whenPressed(new InstantCommand(() -> {
            grabber.setPosition(GrabberComponent.GrabberPosition.CLOSED);
        }));

        Inputs.gamepad1.getGamepadButton(OPEN_GRABBER_BUTTON).whenPressed(new InstantCommand(() -> {
            grabber.setPosition(GrabberComponent.GrabberPosition.OPEN);
        }));

        turnModeSwitch = Inputs.gamepad1.getGamepadButton(TOGGLE_TURN_MODE_BUTTON);
        turnModeSwitch.whenReleased(new InstantCommand(() -> {
            // Snap the desired angle to nearest cardinal direction and rotate 180 deg
            // this is because we usually use relative turning to aim at a pole, and will need to turn around to
            // get another cone
            drive.desiredDirection = HDriveWrapper.snapAngle(drive.getAngleDeg());
        }));


        // Lift
        Inputs.gamepad1.getGamepadButton(SET_LIFT_TO_GROUND_BUTTON).whenPressed(new InstantCommand(() -> {
            lift.setTargetPosition(LiftComponent.LiftPosition.GROUND);
        }));
        Inputs.gamepad1.getGamepadButton(SET_LIFT_TO_LOW_BUTTON).whenPressed(new InstantCommand(() -> {
            lift.setTargetPosition(LiftComponent.LiftPosition.LOW);
        }));
        Inputs.gamepad1.getGamepadButton(SET_LIFT_TO_MIDDLE_BUTTON).whenPressed(new InstantCommand(() -> {
            lift.setTargetPosition(LiftComponent.LiftPosition.MIDDLE);
        }));
        Inputs.gamepad1.getGamepadButton(SET_LIFT_TO_HIGH_BUTTON).whenPressed(new InstantCommand(() -> {
            lift.setTargetPosition(LiftComponent.LiftPosition.HIGH);
        }));
    }

    public void every_tick(){
        MultipleTelemetry t = TelemetryContainer.getTelemetry();

        // Drivetrain
        // Note: directions of x and y on joystick different to directions on the field
        double strafe =  -Inputs.gamepad1.getRightY();
        double forward =  -Inputs.gamepad1.getRightX();
        // The direction of the left joystick is the desired direction of heading
        double x_joystick_turn = Inputs.gamepad1.getLeftX();
        double y_joystick_turn = Inputs.gamepad1.getLeftY();

        // If not pressed, snap to the nearest cardinal direction
        if (turnModeSwitch.get()) {
            drive.fieldOrientedDriveRelativeRotation(strafe, forward, -RELATIVE_TURN_SPEED_MULTIPLIER * x_joystick_turn);
        } else {
            drive.setTurnDirectionSnap(x_joystick_turn, y_joystick_turn);
            drive.fieldOrientedDriveAbsoluteRotation(strafe, forward);
        }

        t.addData("Angle", new Vector2d(y_joystick_turn, -x_joystick_turn).angle());
        t.addData("Test", "Wireless Debugging");

        // Lift
        lift.run();
    }
}
