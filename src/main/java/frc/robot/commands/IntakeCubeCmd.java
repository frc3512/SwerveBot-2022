package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
//use encoders to figure out how to shoot cone curretly and cube
//use encoders to get rpm

//left intake cone 
/*
 * //testing intake
    s.set(m_robotContainer.driver.getRightTriggerAxis() - m_robotContainer.driver.getLeftTriggerAxis());
    intake intake cube is positive
 */
//need pid to for encoders
public class IntakeCubeCmd extends CommandBase{
    private final IntakeSubsystem intakeSubsystem; 
    public IntakeCubeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem; 
    
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotor(0); 
        System.out.println("IntakeCubeCmd ended"); 
    }

    @Override
    public void execute() {
        intakeSubsystem.setMotor(Constants.IntakeConstants.cubeIntakeSpeed); 
        
    }


    @Override
    public void initialize() {
        System.out.println("IntakeSetCmd started");
        
    }

    @Override
    public boolean isFinished() {
        if(intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.theoreticalStallCurrent){
            return true; 
        }
        return false; 
    }
    
    
}
//left intake cone 