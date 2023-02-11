package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCubeCmd extends CommandBase {
    
    IntakeSubsystem intakeSubsystem; 

    public OuttakeCubeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem; 
        addRequirements(intakeSubsystem);
    }
    @Override
    public void end(boolean interrupted){
        intakeSubsystem.setMotor(0);
        System.out.println("OuttakeCubeCmd ended"); 
    }
    @Override
    public void execute(){
        intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
    }
    @Override
    public void initialize(){
        System.out.println("OuttakeCubeCmd started"); 

    }
    @Override
    public boolean isFinished(){
        if(intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.theoreticalStallCurrent){
            return true;
        }
        return false; 
    }
}

