package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionWrapper;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class AutoCenter extends CommandBase {
  private Swerve s_Swerve;
  private PIDController pidControllerDrive;
  private PIDController pidControllerAngle;
  private PhotonVisionWrapper s_PhotonVisionWrapper;

  

  public AutoCenter(Swerve s_Swerve, PhotonVisionWrapper s_PhotonVisionWrapper){
    this.s_Swerve = s_Swerve;
    this.s_PhotonVisionWrapper = s_PhotonVisionWrapper;

    pidControllerDrive = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
    pidControllerDrive.setTolerance(.3);
    pidControllerDrive.setSetpoint(0);

    pidControllerAngle = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
    pidControllerAngle.setTolerance(2);
    pidControllerAngle.setSetpoint(0);

    

  }

  @Override
  public void execute() {
    if(s_PhotonVisionWrapper.getClosestAprilTag() != null){
        double translationValX = pidControllerDrive.calculate(MathUtil.clamp(s_PhotonVisionWrapper.getClosestAprilTag().getBestCameraToTarget().getX()-1, Constants.Swerve.autoCenterLimit,-Constants.Swerve.autoCenterLimit));
        double translationValY = pidControllerDrive.calculate(MathUtil.clamp(s_PhotonVisionWrapper.getClosestAprilTag().getBestCameraToTarget().getY(),Constants.Swerve.autoCenterLimit,-Constants.Swerve.autoCenterLimit));
        double translationAngle = pidControllerAngle.calculate(s_PhotonVisionWrapper.getClosestAprilTag().getYaw());

        s_Swerve.drive(new Translation2d(translationValX, translationValY), translationAngle, true, false);
      }
    }
}