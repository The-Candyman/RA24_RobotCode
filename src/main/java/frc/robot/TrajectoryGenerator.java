package frc.robot;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.RARHolonomicDriveController;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class TrajectoryGenerator {
  private Pose2d m_initialPose;
  private Pose2d m_finalPose;

  private LocalADStar m_pathFinder;

  private PathConstraints m_pathConstraints = new PathConstraints(
    RobotConstants.config.SwerveDrive.k_maxSpeed,
    RobotConstants.config.SwerveDrive.k_maxLinearAcceleration,
    RobotConstants.config.SwerveDrive.k_maxAngularSpeed,
    RobotConstants.config.SwerveDrive.k_maxAngularAcceleration);

  private RARHolonomicDriveController m_driveController = new RARHolonomicDriveController(
    new PIDConstants(RobotConstants.config.AutoAim.Translation.k_P,
        RobotConstants.config.AutoAim.Translation.k_I,
        RobotConstants.config.AutoAim.Translation.k_D),
    new PIDConstants(
        RobotConstants.config.AutoAim.Rotation.k_P,
        RobotConstants.config.AutoAim.Rotation.k_I,
        RobotConstants.config.AutoAim.Rotation.k_D),
    RobotConstants.config.Auto.k_maxModuleSpeed,
    Units.inchesToMeters(Math.sqrt(2) * (RobotConstants.config.Robot.k_width / 2)));

  public TrajectoryGenerator(Pose2d initialPose, Pose2d finalPose) {
    m_initialPose = initialPose;
    m_finalPose = finalPose;
  }

  public void generatePath() {
    m_pathFinder = new LocalADStar();

    m_pathFinder.setStartPosition(m_initialPose.getTranslation());
    m_pathFinder.setGoalPosition(m_finalPose.getTranslation());
  }

  public void runTrajectory(PathPlannerTrajectory trajectory) {
    SwerveDrive swerve = SwerveDrive.getInstance();
    Timer timer = new Timer();

    timer.start();

    boolean isFinished = false;
    while (!isFinished) {
      State goal = trajectory.sample(timer.get());
      Pose2d currentPose = swerve.getPose();

      goal.targetHolonomicRotation = getRotationProvider(goal.targetHolonomicRotation.getDegrees());

      ChassisSpeeds speeds = m_driveController.calculateRobotRelativeSpeeds(currentPose, goal);

      swerve.drive(speeds);

      isFinished = timer.get() >= trajectory.getTotalTimeSeconds();
    }
  }

  public Optional<PathPlannerPath> getCurrentPath() {
    return Optional.ofNullable(
      m_pathFinder.getCurrentPath(
        m_pathConstraints,
        new GoalEndState(0, m_finalPose.getRotation())));
  }

  public PathPlannerTrajectory pathToTrajectory(PathPlannerPath path) {
    return path.getTrajectory(
      new ChassisSpeeds(),
      SwerveDrive.getInstance().getRotation2d());
  }

  private Rotation2d getRotationProvider(double targetRotation) {
    return new Rotation2d(Math.toRadians(targetRotation));
  }
}
