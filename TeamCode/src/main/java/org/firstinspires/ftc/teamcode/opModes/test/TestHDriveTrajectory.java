package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
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
public class TestHDriveTrajectory extends AutonomousModeBase { //TODO: review

    private RevIMU imu;
    private Pose2d m_pose;
    private DifferentialDriveOdometry m_odometry;
    private Trajectory m_trajectory;
    private RamseteController controller;
    private Timing.Timer timer;
    private DifferentialDriveKinematics m_kinematics;
    private DifferentialDrive m_drive;
    private MotorGroup myMotors1;
    private MotorGroup myMotors2;

    // DETERMINE THESE WITH TESTING WITH ACTUAL DRIVETRAIN
    // TODO: Insert PID proportional
    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    private double timeBeforeLastTick;

    public void setup(){
        timer = new Timing.Timer(30);
        timer.start();

        // gyro
        imu = new RevIMU(hardwareMap);
        imu.init();

        myMotors1 = new MotorGroup(HardwareMapContainer.motor0);
        myMotors2 = new MotorGroup(HardwareMapContainer.motor1);

        m_drive = new DifferentialDrive(myMotors1, myMotors2);

//        // Defining wheel locations relative to the center.
//        Translation2d m_frontLeftLocation =
//                new Translation2d(0.381, 0.381);
//        Translation2d m_frontRightLocation =
//                new Translation2d(0.381, -0.381);
//        Translation2d m_backLeftLocation =
//                new Translation2d(-0.381, 0.381);
//        Translation2d m_backRightLocation =
//                new Translation2d(-0.381, -0.381);

        // Creating my kinematics object using the wheel locations.
        m_kinematics = new DifferentialDriveKinematics(
            0.5
        );

        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
        // TODO: Look at game documentation to figure out what to write for this
        // The position on the field will change because we will end up on both sides, so we need two?
        m_odometry = new DifferentialDriveOdometry(
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

        // Pretty sure we don't need to touch this, I hope
        controller = new RamseteController(2, 0.7);
    }


    @Override
    public void every_tick() {

        // Update the odometry in the periodic block
        // Also update the Field2D object (so that we can visualize this in sim)
        Trajectory.State goal = m_trajectory.sample(timer.elapsedTime()); // sample the trajectory at 3.4 seconds from the beginning
        ChassisSpeeds adjustedSpeeds = controller.calculate(m_pose, goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(adjustedSpeeds);

        final double leftFeedforward = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

        final double leftOutput =
                m_leftPIDController.calculate(myMotors1.getCurrentPosition(), wheelSpeeds.leftMetersPerSecond);
        final double rightOutput =
                m_rightPIDController.calculate(myMotors2.getCurrentPosition(), wheelSpeeds.rightMetersPerSecond);
        myMotors1.set(leftOutput + leftFeedforward);
        myMotors2.set(rightOutput + rightFeedforward);

//        m_drive.arcadeDrive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond,false);

        // Get my gyro angle.
        Rotation2d gyroAngle = Rotation2d.fromDegrees(this.imu.getHeading());

        // Update the pose
        // Not sure if timer.elapsedTime() is how you're supposed to do it, test
        m_pose = m_odometry.update(gyroAngle, wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        timeBeforeLastTick = timer.elapsedTime();

    }

    public void resetOdometry(Pose2d pose) {
        // Not sure if needed, here anyways
        m_odometry.resetPosition(
                pose, this.imu.getRotation2d()
        );
    }

    public Trajectory generateTrajectory(Pose2d sideStart, Pose2d crossScale,ArrayList interiorWaypoints) {
        // pretty sure we have to figure this out
        // TODO: insert maxVelocityMetersPerSecond and maxAcclerationMetersPerSecondSq
        TrajectoryConfig config = new TrajectoryConfig(12, 12);
        config.setReversed(true);

        return TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }
}