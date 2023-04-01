package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

import java.util.ArrayList;

// This should serve as a basis for moving the robot around in Autonomous with fancy trajectories.
// Note this is for Mecanum, will change later
// Could figure out some stuff with looking at docs and entering in measurements, but I don't feel like it.

@Autonomous(name="Mecanum Testing Trajectory", group="test")
@Disabled
public class TestMecanumTrajectory extends AutonomousModeBase { //TODO: Test

    private RevIMU imu;
    private Pose2d m_pose;
    private MecanumDriveOdometry m_odometry;
    private Trajectory m_trajectory;
    private RamseteController controller;
    private Timing.Timer timer;
    private MecanumDriveKinematics m_kinematics;
    private MecanumDrive meca_drive;

    private double timeBeforeLastTick;

    public void setup(){

        timer.start();

        // gyro
        this.imu = new RevIMU(hardwareMap);
        this.imu.init();

        this.meca_drive = new MecanumDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor2, HardwareMapContainer.motor3);

        // Defining wheel locations relative to the center.
        Translation2d m_frontLeftLocation =
                new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation =
                new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation =
                new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation =
                new Translation2d(-0.381, -0.381);

        // Creating my kinematics object using the wheel locations.
        m_kinematics = new MecanumDriveKinematics(
            m_frontLeftLocation,
            m_frontRightLocation,
            m_backLeftLocation,
            m_backRightLocation
        );

        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
        m_odometry = new MecanumDriveOdometry(
            m_kinematics,
            this.imu.getRotation2d(),
            new Pose2d(5.0, 13.5, new Rotation2d())
        );

        // This is left here as a example for what to do.
        Pose2d start = new Pose2d(1.54, 23.23,
        Rotation2d.fromDegrees(-180));
        Pose2d end = new Pose2d(23.7, 6.8,
        Rotation2d.fromDegrees(-160));
        ArrayList interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(14.54, 23.23));
        interiorWaypoints.add(new Translation2d(21.04, 18.23));

        m_trajectory = generateTrajectory(start, end, interiorWaypoints);
        controller = new RamseteController(2, 0.7);
    }


    @Override
    public void every_tick() {

        // Update the odometry in the periodic block
        // Also update the Field2D object (so that we can visualize this in sim)
        Trajectory.State goal = m_trajectory.sample(timer.elapsedTime()); // sample the trajectory at 3.4 seconds from the beginning
        ChassisSpeeds adjustedSpeeds = controller.calculate(m_pose, goal);
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(adjustedSpeeds);
        meca_drive.driveRobotCentric(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond,false);

        // Get my gyro angle.
        Rotation2d gyroAngle = Rotation2d.fromDegrees(this.imu.getHeading());

        // Update the pose
        // Not sure if timer.elapsedTime() is how you're supposed to do it, test
        m_pose = m_odometry.updateWithTime(timer.elapsedTime()-timeBeforeLastTick, gyroAngle, wheelSpeeds);
        timeBeforeLastTick = timer.elapsedTime();

    }

    public void resetOdometry(Pose2d pose) {
        // Not sure if needed, here anyways
        m_odometry.resetPosition(
                pose, this.imu.getRotation2d()
        );
    }

    public Trajectory generateTrajectory(Pose2d sideStart, Pose2d crossScale,ArrayList interiorWaypoints) {
        
        TrajectoryConfig config = new TrajectoryConfig(12, 12);
        config.setReversed(true);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
        return trajectory;
    }
}