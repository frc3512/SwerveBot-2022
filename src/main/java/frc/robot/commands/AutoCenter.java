package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionWrapper;
import frc.robot.subsystems.Swerve;

public class AutoCenter extends CommandBase {
  private Swerve s_Swerve;
  private PIDController pidControllerDriveX;
  private PIDController pidControllerDriveY;
  private PIDController pidControllerAngle;
  private PhotonVisionWrapper s_PhotonVisionWrapper;

  /**
<<<<<<< Updated upstream
   * 
   * @param s_Swerve
   * @param s_PhotonVisionWrapper
   */


  public AutoCenter(Swerve s_Swerve, PhotonVisionWrapper s_PhotonVisionWrapper){
    this.s_Swerve = s_Swerve;
    this.s_PhotonVisionWrapper = s_PhotonVisionWrapper;

    pidControllerDriveX = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
    pidControllerDriveX.setTolerance(.15);
    pidControllerDriveX.setSetpoint(1.2);

    pidControllerDriveY = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
    pidControllerDriveY.setTolerance(.3);
    pidControllerDriveY.setSetpoint(0);

    pidControllerAngle = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
    pidControllerAngle.setTolerance(1);
    pidControllerAngle.setSetpoint(0);
  }

  /**
   * This is called repeatedly while command scheduled. 
   * It drives the robot a set distance away from the april tag by finding the translation values 
   * for the x and y distances, and the angle from the april tag to create a 
   * 2d translation for the swerve subsystem. The translation value for x is the frward distance from the 
   * limelight to the april tag and the y distance is the left value. The x translation value is -1 so that the robot
   * drives to a spot 1 meter away from the april tag. 
   */
  @Override

  public void execute() {
    PhotonTrackedTarget target = s_PhotonVisionWrapper.getClosestAprilTag();
    if(target != null){
        Transform3d targetLocation = target.getBestCameraToTarget();
        double translationValX = MathUtil.clamp(pidControllerDriveX.calculate(targetLocation.getX()),-Constants.Swerve.autoCenterLimit,Constants.Swerve.autoCenterLimit);
        double translationValY = MathUtil.clamp(pidControllerDriveY.calculate(targetLocation.getY()),-Constants.Swerve.autoCenterLimit,Constants.Swerve.autoCenterLimit);
        double translationAngle = pidControllerAngle.calculate(target.getYaw());
        s_Swerve.drive(new Translation2d(translationValX, translationValY), translationAngle, false, false);
      }


    }

    /* Temporary Auto Disable */
    @Override
    public boolean isFinished() {
      // TODO Auto-generated method stub
      return true;
    }
}