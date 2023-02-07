package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax elevatorMotorLeft;
    private final CANSparkMax elevatorMotorRight;

    private final CANCoder elevatorLeftController;
    private final CANCoder elevatorRightController;

    public Elevator() {
        elevatorMotorLeft = new CANSparkMax(Constants.Elevator.motorLeftId, MotorType.kBrushless);
        elevatorLeftController = new CANCoder(Constants.Elevator.canConderLeftId);

        elevatorMotorRight = new CANSparkMax(Constants.Elevator.motorRightId, MotorType.kBrushless);
        elevatorRightController = new CANCoder(Constants.Elevator.canConderRightId);
    }
}