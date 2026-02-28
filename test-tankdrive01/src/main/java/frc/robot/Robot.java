// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;


// from chat
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final LTVUnicycleController m_feedback = new LTVUnicycleController(0.020);
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;

  // from chat
  public static Pose2d m_robotPose = new Pose2d();

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, Rotation2d.kZero),
            List.of(),
            new Pose2d(6, 4, Rotation2d.kZero),
            new TrajectoryConfig(2, 2));
  }


  // from chat
  @Override
    public void robotInit() {
      SmartDashboard.putNumber("Matt_test", 405.09);
        Logger.recordMetadata("ProjectName", "DiffDriveSim");
    }


  @Override
  public void robotPeriodic() {
    m_drive.periodic();
    // from chat
    // log values
        Logger.recordOutput("RobotMatt/Pose", m_robotPose);
        SmartDashboard.putNumber("MattRobotX", m_robotPose.getX());
    SmartDashboard.putNumber("MattRobotY", m_robotPose.getY());
    SmartDashboard.putNumber("MattRobotRotationDeg", m_robotPose.getRotation().getRadians());
    SmartDashboard.putNumberArray("MattRobot3", new double[] {m_robotPose.getX(), m_robotPose.getY(), m_robotPose.getRotation().getRadians()});
        // Logger.recordOutput("Robot/GyroDeg", m_gyro.getRotation2d().getDegrees());
  }

  @Override
  public void autonomousInit() {
    m_timer.restart();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_feedback.calculate(m_drive.getPose(), reference);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, rot);
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
}
