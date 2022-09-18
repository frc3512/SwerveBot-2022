package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase {

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private Swerve s_Swerve;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  /** Driver control */
  public TeleopSwerve(
      Swerve s_Swerve,
      Joystick controller,
      int translationAxis,
      int strafeAxis,
      int rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  @Override
  public void execute() {
    double yAxis = -controller.getRawAxis(translationAxis);
    double xAxis = -controller.getRawAxis(strafeAxis);
    double rAxis = -controller.getRawAxis(rotationAxis);
    
    /* Deadbands */
    yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis * 0.5;
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis * 0.5;
    rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis * 0.5;

    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }
}
