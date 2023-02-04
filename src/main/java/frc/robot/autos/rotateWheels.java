package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class rotateWheels extends CommandBase {
    /**
     * 
     * @param s_Swerve 
     * @param degrees the rotation for swerve
     */
    public rotateWheels(Swerve s_Swerve, Rotation2d degrees) {
        addRequirements(s_Swerve);

        s_Swerve.setModuleRotation(degrees);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
