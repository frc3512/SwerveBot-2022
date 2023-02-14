package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeConeCmd extends CommandBase {
    
    private final IntakeSubsystem intakeSubsystem; 
    Timer time;

    public IntakeConeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem; 
        this.time = new Timer();
        addRequirements(intakeSubsystem);
    }
    @Override
    public void end(boolean interrupted){
        time.stop();
        time.reset();
        intakeSubsystem.setMotor(0);
        System.out.println("IntakeConeCmd ended"); 
    }
    @Override
    public void execute(){
        intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
    }
    @Override
    public void initialize(){
        time.start();
        System.out.println("IntakeConeCmd started"); 

    }
    @Override
    public boolean isFinished(){
        if(intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake && time.hasElapsed(.3)){
            return true;
        }
        return false; 
    }
}

