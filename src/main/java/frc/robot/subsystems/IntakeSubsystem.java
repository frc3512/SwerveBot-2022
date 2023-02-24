package frc.robot.subsystems;



import com.revrobotics.CANEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class IntakeSubsystem extends SubsystemBase {
    // private final CANSparkMax wristMotor;
    private final CANSparkMax intakeMotor; 
    private final RelativeEncoder outtakEncoder; 
    
    
    
    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless); 
        // wristMotor = new CANSparkMax(Constants.IntakeConstants.wristMotorId, MotorType.kBrushless);
        // pdm = new PowerDistribution(1, PowerDistribution.ModuleType.kRev); 
        outtakEncoder = intakeMotor.getEncoder(); 
        
        outtakEncoder.setPositionConversionFactor(IntakeConstants.kDriveEncoderRot2Meter); 
        outtakEncoder.setVelocityConversionFactor(IntakeConstants.kDriveEncoderRPM2MeterPerSec); 
        
        resetEncoders(); 
        
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
    public void resetEncoders(){
        outtakEncoder.setPosition(0); 
        
    }
    
}