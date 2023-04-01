
package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

// The gyro stuff was very ill done, no clue if it would work or not

@TeleOp(name="H-Drive Drivetrain", group="Test")
@Disabled
public class TestHDriveDrivetrain extends TeleOpModeBase { //TODO: replace trackWidthMeters, peer review, test
    private DifferentialDrive m_drive;
    private RevIMU imu;
    private DifferentialDriveKinematics m_kinematics;

    @Override
    public void setup() {

        //groups motors together for drive

        imu = new RevIMU(hardwareMap);
        imu.init();

        m_kinematics = new DifferentialDriveKinematics(15.0 / 254.0);

        MotorGroup myMotors1 = new MotorGroup(HardwareMapContainer.motor0);
        MotorGroup myMotors2 = new MotorGroup(HardwareMapContainer.motor1);
        m_drive = new DifferentialDrive(myMotors1, myMotors2);
        HardwareMapContainer.motor2.setRunMode(Motor.RunMode.RawPower);

        //some of this is unneeded, can remove at a later time
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void every_tick() {
        // Code to run in a loop after `PLAY` is pressed
        telemetry.addData("Status", "Running");
        telemetry.addData("X", Inputs.gamepad1.getLeftX());
        telemetry.addData("Y", Inputs.gamepad1.getLeftY());
        telemetry.update();
        //p sure the test motor thing is useless so I deleted it
        m_drive.arcadeDrive(Inputs.gamepad1.getLeftY(), Inputs.gamepad1.getLeftX());
        double correctHeading = 0;
        if (!(Inputs.gamepad1.getLeftY() == 0 && Inputs.gamepad1.getLeftX() == 0)) {
            correctHeading = imu.getHeading();
        }

// set the proportional output power of the motor
        HardwareMapContainer.motor2.set(Inputs.gamepad1.getLeftX());
        double differenceInHeading = imu.getHeading() - correctHeading;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, differenceInHeading);
        DifferentialDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(chassisSpeeds);
        m_drive.arcadeDrive(Inputs.gamepad1.getLeftY()+wheelSpeeds.leftMetersPerSecond, Inputs.gamepad1.getLeftX()+wheelSpeeds.rightMetersPerSecond);
    }
}

