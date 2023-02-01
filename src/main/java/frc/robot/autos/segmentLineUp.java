package frc.robot.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class segmentLineUp extends SequentialCommandGroup {
    public enum SEGMENT { // Numbers in order of segment from left to right (driver station POV)
        CONE_1,
        CONE_2,
        CONE_3,
        CONE_4,
        CONE_5,
        CONE_6,
        CUBE_1,
        CUBE_2,
        CUBE_3
    }

    public segmentLineUp(Swerve s_Swerve, SEGMENT segment, Supplier<PathPoint> startPoint) {
        PathPoint lineUpPoint = startPoint.get();

        switch (segment) {
            case CONE_1:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 4.93),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CONE_2:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 3.89),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CONE_3:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 3.25),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CONE_4:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 2.2),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CONE_5:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 1.6),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CONE_6:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, .47),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CUBE_1:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 4.43),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CUBE_2:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 2.74),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
            case CUBE_3:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.75, 1.05),
                        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90));
                break;
        }

        PathPlannerTrajectory trajectoryToSegment = PathPlanner.generatePath(
                Constants.AutoConstants.constraints,
                startPoint.get(),
                lineUpPoint);


        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectoryToSegment, DriverStation.getAlliance());

        addCommands(
                new executeTrajectory(s_Swerve, trajectoryToSegment));
    }
}
