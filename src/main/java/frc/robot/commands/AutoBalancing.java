package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalancing extends CommandBase {
  private Swerve s_Swerve;
  private PIDController pidController;

  /**
   * 
   * @param s_Swerve TODO
   */

  public AutoBalancing(Swerve s_Swerve){
    this.s_Swerve = s_Swerve;
    pidController = new PIDController(
        Constants.Swerve.drivePitchKP, 
        Constants.Swerve.drivePitchKI, 
        Constants.Swerve.drivePitchKD, 
        Constants.Swerve.drivePitchKFF);
    pidController.setTolerance(1);
    pidController.setSetpoint(Constants.Swerve.pitchSetPoint);
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("At Tolerance", pidController.atSetpoint());
        double translationVal  = MathUtil.clamp(
            pidController.calculate(
                s_Swerve.getPitch().getDegrees(), Constants.Swerve.pitchSetPoint), -.35, .35);
        s_Swerve.drive(
        new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        // SwerveModule[] modules = s_Swerve.getModules();
        // modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0), true);
        // modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0), true);
        // modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0), true);
        // modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0), true);
    }

    @Override
    public boolean isFinished() {
        /* Temporary Auto disable */
        return true;
        // SmartDashboard.putBoolean("At Tolerance", pidController.atSetpoint());
        // return pidController.atSetpoint();
    }
}