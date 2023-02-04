package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class Swerve extends SubsystemBase {
  private Pigeon2 gyro;

  private SwerveDrivePoseEstimator swervePoseEstimator;
  private SwerveModule[] mSwerveMods;

  private PhotonVisionWrapper pcw;

  private Field2d field;

  /**
   * TODO
   */
  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    zeroGyro();

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(),
        new Pose2d());

    pcw = new PhotonVisionWrapper();

    field = new Field2d();
    SmartDashboard.putData(field);
  }
  /**
   * 
   * @param translation
   * @param rotation
   * @param fieldRelative
   * @param isOpenLoop
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Used by SwerveControllerCommand in Auto
   * @param desiredStates
   */
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }
  /**
   * 
   * @param rotation
   */
  public void setModuleRotation(Rotation2d rotation) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(new SwerveModuleState(0, rotation), false);
    }
  }

  /**
   * 
   * @return Pose2d
   */
  public Pose2d getPose() {
    Pose2d pose = swervePoseEstimator.getEstimatedPosition();
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      Translation2d transformedTranslation =
          new Translation2d(FieldConstants.fieldLength - pose.getX(), FieldConstants.fieldWidth - pose.getY());

      Rotation2d transformedRotation = pose.getRotation().plus(Rotation2d.fromDegrees(180));
    return new Pose2d(transformedTranslation,transformedRotation);
    }
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * 
   * @return Field2d
   */
  public Field2d getField() {
    return field;
  }

  /**
   * 
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  /**
   * 
   */
  public void resetToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * 
   * @return SwerveModuleState[]
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * 
   * @return SwerveModulePosition[]
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /**
   * 
   */
  public void zeroGyro() {
    gyro.setYaw(0);
  }
  /**
   * 
   * @return Rotation2d
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(gyro.getPitch());
  }

  /**
   * 
   * @return PathPoint
   */
  public PathPoint getPoint() {
    return new PathPoint(getPose().getTranslation(), getPose().getRotation());
  }

  /**
   * 
   * @return Rotation2d
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  /**
   * 
   * @return PhotonVisionWrapper
   */
  public PhotonVisionWrapper getCamera(){
    return pcw;
  }

  /**
   * 
   */
  @Override
  public void periodic() {
        swervePoseEstimator.update(getYaw(), getPositions());
        Optional<EstimatedRobotPose> result =
                pcw.getEstimatedGlobalPose(getPose());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            swervePoseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }

        field.setRobotPose(getPose());

    SmartDashboard.putNumber("Pigeon2 Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Pigeon2 Pitch", getPitch().getDegrees());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
