package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.sensors.Pigeon2;

public class AutoBalancing extends CommandBase {
  private Swerve s_Swerve;
  private PIDController pidController;

  public AutoBalancing(Swerve s_Swerve){
    this.s_Swerve = s_Swerve;
    pidController = new PIDController(
        Constants.Swerve.drivePitchKP, 
        Constants.Swerve.drivePitchKI, 
        Constants.Swerve.drivePitchKD, 
        Constants.Swerve.drivePitchKFF);
    pidController.setTolerance(1);
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
        double translationVal  = MathUtil.clamp(
            pidController.calculate(
                -s_Swerve.getPitch().getDegrees(), Constants.Swerve.pitchSetPoint), -1, 1);
        s_Swerve.drive(
        new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
    }

public boolean isFinished() {
    return pidController.atSetpoint();
    }
}