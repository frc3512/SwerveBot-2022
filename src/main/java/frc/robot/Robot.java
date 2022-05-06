// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final DrivetrainSubsystem m_swerve = new DrivetrainSubsystem();
  private final XboxController m_driveController = new XboxController(0);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    driveWithJoysticks(false);
  }

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithJoysticks(true);

    if(m_driveController.getAButtonPressed())
    {
      m_swerve.zeroGyroscope();
    }

    m_swerve.updateOdometry();
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  private void driveWithJoysticks(boolean fieldRelative)
  {
    double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getLeftY(), 0.1))
      * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.5;

    double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getLeftX(), 0.1))
      * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.5;

    double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRightX(), 0.02))
      * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
