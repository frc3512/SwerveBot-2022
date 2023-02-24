package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends CommandBase {
    
    boolean intake; 
    boolean cone; 
    private final IntakeSubsystem intakeSubsystem; 
    Timer time;
    
    public IntakeCmd(IntakeSubsystem intakeSubsystem, boolean intake, boolean cone){
        this.intakeSubsystem = intakeSubsystem; 
        
        this.time = new Timer();
        
        
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        time.start();
        
        System.out.println("IntakeCmd started"); 
        
    }
    @Override
    public void execute(){
        if(cone && intake){
            intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
        }
        if(!cone && intake){
            intakeSubsystem.setMotor(Constants.IntakeConstants.cubeIntakeSpeed);
            
        }
        if(cone && !intake){
            intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
        }
        if(!cone && !intake){
            intakeSubsystem.setMotor(Constants.IntakeConstants.cubeIntakeSpeed);
            
        }
        
    }
   
    @Override
    public void end(boolean interrupted){
        
            time.stop();
            time.reset();
        
        intakeSubsystem.setMotor(0);
        System.out.println("IntakeCmd ended"); 
    }
    
    @Override
    public boolean isFinished(){
        if(cone && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake && time.hasElapsed(.3)){
            return true;
        }
        if(!cone && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake  && time.hasElapsed(.3) ){
            return true; 
        }
        if(!intake){
            return time.hasElapsed(.5); 
        }
        return false; 
    }
}

