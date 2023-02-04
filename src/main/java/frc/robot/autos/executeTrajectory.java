package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class executeTrajectory extends SequentialCommandGroup {
  public executeTrajectory(Swerve s_Swerve, PathPlannerTrajectory trajectory, boolean setInitialPose) {

    s_Swerve.getField().getObject("Field").setTrajectory(trajectory);

    PIDController thetaController = new PIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

    if(setInitialPose){
    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialHolonomicPose())),
        swerveControllerCommand);
    } else{
        addCommands(
            swerveControllerCommand);
    }
  }
}
