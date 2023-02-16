package frc.robot.subsystems;



import com.revrobotics.CANEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class IntakeSubsystem extends SubsystemBase {
    // private final CANSparkMax wristMotor;
    private final CANSparkMax intakeMotor; 
    private final RelativeEncoder intakeEncoder = new RelativeEncoder() {
        
    };; 


    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless); 
        // wristMotor = new CANSparkMax(Constants.IntakeConstants.wristMotorId, MotorType.kBrushless);
        // pdm = new PowerDistribution(1, PowerDistribution.ModuleType.kRev); 
    }
    public void setMotor(double speed){
        intakeMotor.set(speed); 
    }
     public double getPDMCurrent(){
         return intakeMotor.getOutputCurrent(); 
    }
    @Override 
    public void periodic(){
        //returns in amps
       // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel); 
       double intakeCurrent = intakeMotor.getOutputCurrent();  
        SmartDashboard.putNumber("Intake Current", intakeCurrent); 
    }  
}