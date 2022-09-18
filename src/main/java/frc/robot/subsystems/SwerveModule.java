package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private boolean driveInvert;
  private boolean angleInvert;

  private double angleOffset;
  private double lastAngle;
  private CANCoder angleEncoder;
  private CANSparkMax mAngleMotor;
  private CANSparkMax mDriveMotor;
  private RelativeEncoder mDriveEncoder;
  private RelativeEncoder mAngleEncoder;

  public static PIDController mDrivePID =
      new PIDController(
          Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
  public static PIDController mAnglePID =
      new PIDController(
          Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);

  private SimpleMotorFeedforward mDriveFeedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;
    driveInvert = moduleConstants.driveInvert;
    angleInvert = moduleConstants.angleInvert;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    mAngleEncoder = mAngleMotor.getEncoder();
    configAngleMotor();

    /* Drive Motor Config */
    mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    mDriveEncoder = mDriveMotor.getEncoder();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      mDriveMotor.set(percentOutput);
    } else {
      double velocity =
          Conversions.MPSToNeo(
              desiredState.speedMetersPerSecond,
              Constants.Swerve.wheelCircumference,
              Constants.Swerve.driveGearRatio);
      mDriveMotor.setVoltage(
          mDrivePID.calculate(mDriveEncoder.getVelocity(), velocity)
              + mDriveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle.getDegrees();
    mAngleMotor.setVoltage(
        mAnglePID.calculate(
            mAngleEncoder.getPosition(),
            Conversions.degreesToNeo(angle, Constants.Swerve.angleGearRatio)));
    lastAngle = angle;
  }

  private void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToNeo(
            getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
    mAngleEncoder.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    double velocity =
        Conversions.neoToMPS(
            mDriveEncoder.getVelocity(),
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveGearRatio);
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.neoToDegrees(mAngleEncoder.getPosition(), Constants.Swerve.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    mAngleMotor.restoreFactoryDefaults();
    mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    mAngleMotor.setInverted(angleInvert);
    mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    mAngleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    mDriveMotor.restoreFactoryDefaults();
    mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    mDriveMotor.setInverted(driveInvert);
    mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    mDriveMotor.burnFlash();
    mDriveEncoder.setPosition(0.0);
  }
}
