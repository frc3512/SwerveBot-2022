package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    //TODO
    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(20.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 6.12:1
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    public static final double pitchSetPoint = 0.0;

    public static final double drivePitchKP = 0.04;
    public static final double drivePitchKI = 0.00005;
    public static final double drivePitchKD = 0.000000000000001;
    public static final double drivePitchKFF = 0.000000000000001;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.11979;
    public static final double driveKV = 2.3823;
    public static final double driveKA = 0.30034;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3; // meters per second
    public static final double maxAngularVelocity = 5;

    /* Swerve Limiting Values */
    public static final double autoCenterLimit = .3;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

  
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 20;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 30;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(238.1835);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 31;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(36.8262);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 32;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(345.2);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 23;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(205.7519);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final PathConstraints constraints = new PathConstraints(1, 1);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
  }

  public static final class PhotonVision{
    public static final String photonVisionName = "OV5647";
    public static final Transform3d robotToCam =
    new Transform3d(
            new Translation3d(Units.inchesToMeters(11.4), 0.0, Units.inchesToMeters(6.4)),
            new Rotation3d(
                    0, 0,
                    0));
  }

  public static final class AprilTags {
    public static final AprilTag tag1 = new AprilTag(1, FieldConstants.aprilTags.get(1));
    public static final AprilTag tag2 = new AprilTag(2, FieldConstants.aprilTags.get(2));
    public static final AprilTag tag3 = new AprilTag(3, FieldConstants.aprilTags.get(3));
    public static final AprilTag tag4 = new AprilTag(4, FieldConstants.aprilTags.get(4));
    public static final AprilTag tag5 = new AprilTag(5, FieldConstants.aprilTags.get(5));
    public static final AprilTag tag6 = new AprilTag(6, FieldConstants.aprilTags.get(6));
    public static final AprilTag tag7 = new AprilTag(7, FieldConstants.aprilTags.get(7));
    public static final AprilTag tag8 = new AprilTag(8, FieldConstants.aprilTags.get(8));
    public static final ArrayList<AprilTag> aprilTagList = new ArrayList<>();

    static {
      aprilTagList.add(tag1);
      aprilTagList.add(tag2);
      aprilTagList.add(tag3);
      aprilTagList.add(tag4);
      aprilTagList.add(tag5);
      aprilTagList.add(tag6);
      aprilTagList.add(tag7);
      aprilTagList.add(tag8);
    }
  }

  public static final class Elevator{
    public static final int motorLeftId = 51;
    public static final int canConderLeftId = 61;

    public static final int motorRightId = 52;
    public static final int canConderRightId = 62;

    
  }
  public static final class IntakeConstants{
    public static final int intakeMotorId = 58; 
    public static final int wristMotorId = 59;
    public static final int pdpChannel = 2; //update number later

    public static final double coneIntakeSpeed = 1; 
    public static final double cubeIntakeSpeed = 1; 

    public static final double outtakeSpeed = 1; 

    public static final double maxCurrentIntake = 80; 

    //wheel diameter, gear ratio, encoder constants
        //will need to change depending on the robot/swerve
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;

  }
}