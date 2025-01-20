package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AutoTrajectory {
  private Pose2d m_initialPose;
  private Pose2d m_finalPose;

  private PathConstraints pathConstraints = new PathConstraints(
    RobotConstants.config.SwerveDrive.k_maxSpeed,
    RobotConstants.config.SwerveDrive.k_maxLinearAcceleration,
    RobotConstants.config.SwerveDrive.k_maxAngularSpeed,
    RobotConstants.config.SwerveDrive.k_maxAngularAcceleration);

  public AutoTrajectory(Pose2d initialPose, Pose2d finalPose) {
    m_initialPose = initialPose;
    m_finalPose = finalPose;
  }

  public PathPlannerPath generatePath() {
    LocalADStar pathFinder = new LocalADStar();

    pathFinder.setStartPosition(m_initialPose.getTranslation());
    pathFinder.setGoalPosition(m_finalPose.getTranslation());

    // wait for a path to be generated given the above parameters
    while (!pathFinder.isNewPathAvailable()) {}

    return pathFinder.getCurrentPath(
      pathConstraints,
      new GoalEndState(0, Rotation2d.fromDegrees(0)));
  }

  public PathPlannerTrajectory pathToTrajectory(PathPlannerPath path) {
    return path.getTrajectory(
      new ChassisSpeeds(),
      SwerveDrive.getInstance().getRotation2d());
  }
}
