package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
    Timer time; 
    
    public IntakeCubeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem; 
        this.time = new Timer(); 
        addRequirements(intakeSubsystem);
    }
    //TODO update cube code to match cones code
    @Override
    public void end(boolean interrupted) {
        time.stop();
        time.reset();
        intakeSubsystem.setMotor(0); 
        System.out.println("IntakeCubeCmd ended"); 
    }

    @Override
    public void execute() {
        intakeSubsystem.setMotor(Constants.IntakeConstants.cubeIntakeSpeed); 
        
    }


    @Override
    public void initialize() {
        time.start(); 
        System.out.println("IntakeSetCmd started");
        
    }

    @Override
    public boolean isFinished() {
        if(intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake){
            return true; 
        }
        return false; 
    }
    
    
}
//left intake cone 