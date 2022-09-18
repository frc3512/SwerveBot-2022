package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final class Swerve {
    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (5.14 / 1.0); // 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int driveContinuousCurrentLimit = 35;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.99;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.10;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorIDMod0 = 20;
      public static final int angleMotorIDMod0 = 10;
      public static final int canCoderIDMod0 = 1;
      public static final double angleOffsetMod0 = 143.43;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorIDMod0, angleMotorIDMod0, canCoderIDMod0, angleOffsetMod0, false, false);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorIDMod1 = 21;
      public static final int angleMotorIDMod1 = 11;
      public static final int canCoderIDMod1 = 2;
      public static final double angleOffsetMod1 = 103.44;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorIDMod1, angleMotorIDMod1, canCoderIDMod1, angleOffsetMod1, false, false);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorIDMod2 = 12;
      public static final int angleMotorIDMod2 = 22;
      public static final int canCoderIDMod2 = 3;
      public static final double angleOffsetMod2 = 148.97;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorIDMod2, angleMotorIDMod2, canCoderIDMod2, angleOffsetMod2, true, false);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorIDMod3 = 13;
      public static final int angleMotorIDMod3 = 23;
      public static final int canCoderIDMod3 = 4;
      public static final double angleOffsetMod3 = 333.10;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorIDMod3, angleMotorIDMod3, canCoderIDMod3, angleOffsetMod3, true, false);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
