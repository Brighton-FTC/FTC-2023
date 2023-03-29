package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;


@TeleOp(name="Test Mecanum Drive [12sliu]", group="Drivetrain")
@Disabled
public class TestMecanumDrive extends TeleOpModeBase { //TODO: Test

    // DRIVE TRAIN
    private MecanumDrive meca_drive;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;
    private boolean fieldCentric = false;
    private RevIMU imu;

    @Override
    public void setup() {

        //groups motors together for drive
        this.meca_drive = new MecanumDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor2, HardwareMapContainer.motor3);
        this.imu = new RevIMU(hardwareMap);
        this.imu.init();
        new GamepadButton(Inputs.gamepad1, PSButtons.TRIANGLE).whenPressed(() -> fieldCentric = !fieldCentric);
        new GamepadButton(Inputs.gamepad1, PSButtons.CIRCLE).whenPressed(() -> isForwardOnlyMode = !isForwardOnlyMode);
        new GamepadButton(Inputs.gamepad1, PSButtons.SQUARE).whenPressed(() -> isSlowMode = !isSlowMode);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void every_tick() {
        // Code to run in a loop after `PLAY` is pressed
        telemetry.addData("Status", "Running");
        telemetry.addData("leftX", Inputs.gamepad1.getLeftX());
        telemetry.addData("leftY", Inputs.gamepad1.getLeftY());
        telemetry.addData("rightX", Inputs.gamepad1.getRightX());
        double leftY = Inputs.gamepad1.getLeftY();
        double leftX = Inputs.gamepad1.getLeftX();
        double rightX = Inputs.gamepad1.getRightX();
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