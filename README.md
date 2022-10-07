# SwerveBot-2022
## Team 3512's swerve drive code for the 2022 offseason
### Written in Java, using WPILib's [Command-Based](https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html) structure
### Modified from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve) project to utilize __NEO motors__

Therefore, their [instructions](https://github.com/Team364/BaseFalconSwerve#setting-constants) on modifiying the constants to fit your robot will also work on this repo as well.

### Configurations
- Works with a swerve drive with __NEO brushless motors__, __Spark Max motor controllers__, __CTRE CANCoders__, and __CTRE Pigeon 2.0 IMU__.
- Drive and steer ratios are configured for the __SDS MK4 L4 modules__, but can be easily be adapted to other modules.
- Positions in the kinematics are configured differently as we flipped our front two modules by 90 physically. Make changes here if you have them configured in the normal "setup"
- We've also cut the speed by 50% in the [TeleopSwerve](https://github.com/frc3512/SwerveBot-2022/blob/main/src/main/java/frc/robot/commands/TeleopSwerve.java) command for our own robot. Definitely change/remove this accordingly, especially when using modules with different drive gear ratios.
