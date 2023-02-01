// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);

  /* Drive Controls */
  private static final int translationAxis = XboxController.Axis.kLeftY.value;
  private static final int strafeAxis = XboxController.Axis.kLeftX.value;
  private static final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  /* Autonomous Mode Chooser */
  private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

  /* Autonomous Modes */
  PathPlannerTrajectory moveForward = PathPlanner.loadPath("Move Forward",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sCurve = PathPlanner.loadPath("S Curve",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sussy = PathPlanner.loadPath("sussy",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> driver.povDown().getAsBoolean(),
            () -> driver.leftBumper().getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();

    // Configure Smart Dashboard options
    configureSmartDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    driver.x().onTrue(new AutoBalancing(s_Swerve));
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  private void configureSmartDashboard() {
    autoChooser.setDefaultOption("Move forward", moveForward);
    autoChooser.addOption("S curve", sCurve);
    autoChooser.addOption("SUSSY - CADEN", sussy);

    SmartDashboard.putData(autoChooser);
  }

  public void disabledInit() {
    s_Swerve.resetToAbsolute();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard
    return new executeTrajectory(s_Swerve, autoChooser.getSelected());
  }
}
