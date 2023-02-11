package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.IntakeCubeCmd;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax wristMotor;
    private final CANSparkMax intakeMotor; 
    private final PowerDistribution pdm;

    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless); 
        wristMotor = new CANSparkMax(Constants.IntakeConstants.wristMotorId, MotorType.kBrushless);
        pdm = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

        new IntakeCubeCmd(this); 
    }
    public void setMotor(double speed){
        intakeMotor.set(speed); 
    }
    public double getPDMCurrent(){
        return pdm.getCurrent(Constants.IntakeConstants.pdpChannel); 
    }
    @Override 
    public void periodic(){
        //returns in amps
       // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel); 
       double intakeCurrent = intakeMotor.getBusVoltage(); 
        SmartDashboard.putNumber("Intake Current", intakeCurrent); 
        
        

    }
    
}
