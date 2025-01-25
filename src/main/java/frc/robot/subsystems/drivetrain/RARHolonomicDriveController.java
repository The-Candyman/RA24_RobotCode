package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;

public class RARHolonomicDriveController extends PPHolonomicDriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController rotationController;
  private final double maxModuleSpeed;
  private final double mpsToRps;

  @SuppressWarnings("unused")
  private Translation2d translationError = new Translation2d();
  private boolean isEnabled = true;

  private static Supplier<Optional<Rotation2d>> rotationTargetOverride = null;

  public RARHolonomicDriveController(
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      double period,
      double maxModuleSpeed,
      double driveBaseRadius) {
    super(translationConstants, rotationConstants, period, maxModuleSpeed, driveBaseRadius);

    this.xController = new PIDController(
        translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    this.xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

    this.yController = new PIDController(
        translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    this.yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

    // Temp rate limit of 0, will be changed in calculate
    this.rotationController = new ProfiledPIDController(
        rotationConstants.kP,
        rotationConstants.kI,
        rotationConstants.kD,
        new TrapezoidProfile.Constraints(0, 0),
        period);
    this.rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    this.maxModuleSpeed = maxModuleSpeed;
    this.mpsToRps = 1.0 / driveBaseRadius;
  }

  public RARHolonomicDriveController(
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      double maxModuleSpeed,
      double driveBaseRadius) {
    this(translationConstants, rotationConstants, 0.02, maxModuleSpeed, driveBaseRadius);
  }

  boolean firstTimeForEverything = true;

  @Override
  public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectory.State targetState) {
    // This is the only thing we actually changed
    if (firstTimeForEverything) {
      firstTimeForEverything = false;
      rotationController.reset(currentPose.getRotation().getRadians());
    }

    double xFF = targetState.velocityMps * targetState.heading.getCos();
    double yFF = targetState.velocityMps * targetState.heading.getSin();

    this.translationError = currentPose.getTranslation().minus(targetState.positionMeters);

    if (!this.isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, 0, currentPose.getRotation());
    }

    double xFeedback = this.xController.calculate(currentPose.getX(), targetState.positionMeters.getX());
    double yFeedback = this.yController.calculate(currentPose.getY(), targetState.positionMeters.getY());

    double angVelConstraint = targetState.constraints.getMaxAngularVelocityRps();
    double maxAngVel = angVelConstraint;

    if (Double.isFinite(maxAngVel)) {
      // Approximation of available module speed to do rotation with
      double maxAngVelModule = Math.max(0, maxModuleSpeed - targetState.velocityMps) * mpsToRps;
      maxAngVel = Math.min(angVelConstraint, maxAngVelModule);
    }

    var rotationConstraints = new TrapezoidProfile.Constraints(
        maxAngVel, targetState.constraints.getMaxAngularAccelerationRpsSq());

    Rotation2d targetRotation = targetState.targetHolonomicRotation;
    if (rotationTargetOverride != null) {
      targetRotation = rotationTargetOverride.get().orElse(targetRotation);
    }

    double rotationFeedback = rotationController.calculate(
        currentPose.getRotation().getRadians(),
        new TrapezoidProfile.State(targetRotation.getRadians(), 0),
        rotationConstraints);
    double rotationFF = targetState.holonomicAngularVelocityRps.orElse(rotationController.getSetpoint().velocity);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
  }

  public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, Pose2d goalPose, double maxApproachSpeed, double maxApproachAngularSpeed) {
    if (firstTimeForEverything) {
      firstTimeForEverything = false;
      rotationController.reset(currentPose.getRotation().getRadians());
    }

    this.translationError = currentPose.getTranslation().minus(goalPose.getTranslation());

    double distance = currentPose.getTranslation().getDistance(goalPose.getTranslation()); // meters i think?

    // rot-diff between our current rot and what we want to be at (0.5 is the max we can be off)
    double rotationError = currentPose.getRotation().minus(goalPose.getRotation()).getRotations();

    // times 2 is the decay rate
    double targetSpeed = Math.min(distance * 2, maxApproachSpeed);
    double targetAngularSpeed = Math.min(rotationError * 2, maxApproachAngularSpeed);

    double xFF = targetSpeed * goalPose.getRotation().getCos();
    double yFF = targetSpeed * goalPose.getRotation().getSin();

    if (!this.isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, 0, currentPose.getRotation());
    }

    double xFeedback = this.xController.calculate(currentPose.getX(), goalPose.getX());
    double yFeedback = this.yController.calculate(currentPose.getY(), goalPose.getY());

    double angVelConstraint = Units.radiansToRotations(RobotConstants.config.SwerveDrive.k_maxAngularSpeed);
    double angAccConstraint = Units.radiansToRotations(RobotConstants.config.SwerveDrive.k_maxAngularAcceleration);
    // double maxAngVel = angVelConstraint;

    if (Double.isFinite(angVelConstraint)) {
      // Approximation of available module speed to do rotation with
      double maxAngVelModule = Math.max(0, maxModuleSpeed - targetSpeed) * mpsToRps;
      angVelConstraint = Math.min(angVelConstraint, maxAngVelModule);
    }

    TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(angVelConstraint, angAccConstraint);

    Rotation2d targetRotation = goalPose.getRotation();
    if (rotationTargetOverride != null) {
      targetRotation = rotationTargetOverride.get().orElse(targetRotation);
    }

    double rotationFeedback = rotationController.calculate(
        currentPose.getRotation().getRadians(),
        new TrapezoidProfile.State(targetRotation.getRadians(), 0),
        rotationConstraints);
    double rotationFF = targetAngularSpeed;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
  }
}
