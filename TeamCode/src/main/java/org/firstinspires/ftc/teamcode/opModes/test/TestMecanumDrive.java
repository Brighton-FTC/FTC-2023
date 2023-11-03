package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name="Test Mecanum Drive [12sliu]", group="Drivetrain")
@Disabled
public class TestMecanumDrive extends OpMode { //TODO: Test

    // DRIVE TRAIN
    private MecanumDrive meca_drive;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;
    private boolean fieldCentric = false;
    private RevIMU imu;

    private GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

    @Override
    public void setup() {
        // TODO: Define PSButtons enum in a different file
//        package org.firstinspires.ftc.teamcode.inputs;
//
//        import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//
//        /**
//         * Gamepad buttons referred to by their PlayStation names. For other buttons, please see {@link GamepadKeys.Button}
//         */
//        public class PSButtons {
//            public static final GamepadKeys.Button SQUARE = GamepadKeys.Button.X;
//            public static final GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
//            public static final GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;
//            public static final GamepadKeys.Button CROSS = GamepadKeys.Button.A;
//
//            public static final GamepadKeys.Button SHARE = GamepadKeys.Button.BACK;
//            public static final GamepadKeys.Button OPTIONS = GamepadKeys.Button.START;
//        }


        //groups motors together for drive
        this.meca_drive = new MecanumDrive(new Motor(hardwareMap, "frontLeftDrive"), new Motor(hardwareMap, "frontRightDrive"), new Motor(hardwareMap, "backLeftDrive"), new Motor(hardwareMap, "backRightDrive"));
        this.imu = new RevIMU(hardwareMap);
        this.imu.init();
        new GamepadButton(gamepadEx1, PSButtons.TRIANGLE).whenPressed(() -> fieldCentric = !fieldCentric);
        new GamepadButton(gamepadEx1, PSButtons.CIRCLE).whenPressed(() -> isForwardOnlyMode = !isForwardOnlyMode);
        new GamepadButton(gamepadEx1, PSButtons.SQUARE).whenPressed(() -> isSlowMode = !isSlowMode);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void every_tick() {
        // Code to run in a loop after `PLAY` is pressed
        telemetry.addData("Status", "Running");
        telemetry.addData("leftX", gamepadEx1.getLeftX());
        telemetry.addData("leftY", gamepadEx1.getLeftY());
        telemetry.addData("rightX", gamepadEx1.getRightX());
        double leftY = gamepadEx1.getLeftY();
        double leftX = gamepadEx1.getLeftX();
        double rightX = gamepadEx1.getRightX();
        if (isSlowMode){
            leftX *= 0.5;
            leftY *= 0.5;
            rightX *= 0.5;
        }
        if (isForwardOnlyMode){
            leftX = 0;
        }
        if (!fieldCentric){
            meca_drive.driveRobotCentric(leftX, leftY, rightX,false);
        }
        else {
            meca_drive.driveFieldCentric(leftX, leftY, rightX, this.imu.getRotation2d().getDegrees(), false);
        }
        telemetry.update();
        //p sure the test motor thing is useless so I deleted it
//        m_drive.arcadeDrive(Inputs.gamepad1.getLeftY(), Inputs.gamepad1.getLeftX());
    }
}