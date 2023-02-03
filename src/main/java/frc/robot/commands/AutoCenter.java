package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionWrapper;
import frc.robot.subsystems.Swerve;

public class AutoCenter extends CommandBase {
  private Swerve s_Swerve;
  private PIDController pidController;
  private PhotonVisionWrapper s_PhotonVisionWrapper;

  public AutoCenter(Swerve s_Swerve, PhotonVisionWrapper s_PhotonVisionWrapper){
    this.s_Swerve = s_Swerve;
    this.s_PhotonVisionWrapper = s_PhotonVisionWrapper;

    pidController = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
    pidController.setTolerance(.1);
  }

  @Override
  public void execute() {  
        double translationValX = pidController.calculate(MathUtil.clamp(Constants.Swerve.autoCenterLimit,-Constants.Swerve.autoCenterLimit,s_PhotonVisionWrapper.getClosestAprilTag().getX()-1));
        double translationValY = pidController.calculate(MathUtil.clamp(Constants.Swerve.autoCenterLimit,-Constants.Swerve.autoCenterLimit,s_PhotonVisionWrapper.getClosestAprilTag().getY()));
        double translationAngle = s_PhotonVisionWrapper.getClosestAprilTag().getRotation().getAngle();

        s_Swerve.drive(new Translation2d(translationValX, translationValY), translationAngle, true, false);
    }
}