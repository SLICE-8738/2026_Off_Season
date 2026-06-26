package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.ShotCalculatorConstants;

/**
 * Centralized shot calculator that provides velocity-compensated virtual targets.
 * Drop-in replacement for the old getVirtualTarget() — subsystems still handle their
 * own auto-goal selection and tracking, but pass targets through here for compensation.
 */
public class ShotCalculator {
    private static ShotCalculator instance;

    private CommandSwerveDrivetrain m_swerveSubsystem;

    // Configuration
    private static final LoggedTunableNumber PHASE_DELAY =
        new LoggedTunableNumber("ShotCalculator/PhaseDelay", 0.02, true);

    // Cached from last calculation (keyed by target position)
    private Translation2d lastTargetPosition = null;
    private double lastCompensatedDistance = 0.0;

    private ShotCalculator() {}

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public void init(CommandSwerveDrivetrain swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
    }

    /**
     * Takes a raw target pose and returns a velocity-compensated virtual target.
     * Accounts for:
     * - Phase delay (projects robot pose forward to compensate for processing latency)
     * - Robot velocity (iteratively offsets aim to lead the target based on air time)
     * - Shooter offset from robot center (rotational velocity cross-term)
     *
     * @param rawTarget The actual target position on the field
     * @return A compensated virtual target Pose2d that the shooter should track
     */
    public Pose2d getVirtualTarget(Pose2d rawTarget) {
        if (m_swerveSubsystem == null) {
            return rawTarget;
        }

        Pose2d robotPose = m_swerveSubsystem.getState().Pose;
        ChassisSpeeds robotRelativeVelocity = m_swerveSubsystem.getState().Speeds;

        // Phase delay compensation — estimate where robot will be
        double phaseDelay = PHASE_DELAY.get();
        Pose2d estimatedPose = robotPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Field-relative velocity
        ChassisSpeeds fieldVelocity = m_swerveSubsystem.getAsFieldRelativeSpeeds();

        // Calculate field-relative shooter velocity including rotational cross-term
        double robotAngle = estimatedPose.getRotation().getRadians();
        double shooterOffsetX = ShotCalculatorConstants.shooterTransform.getX();
        double shooterOffsetY = ShotCalculatorConstants.shooterTransform.getY();

        double shooterVelocityX = fieldVelocity.vxMetersPerSecond
            - fieldVelocity.omegaRadiansPerSecond
                * (shooterOffsetX * Math.sin(robotAngle) + shooterOffsetY * Math.cos(robotAngle));
        double shooterVelocityY = fieldVelocity.vyMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond
                * (shooterOffsetX * Math.cos(robotAngle) - shooterOffsetY * Math.sin(robotAngle));

        Translation2d target = rawTarget.getTranslation();
        Translation2d shooterPosition = estimatedPose
            .transformBy(new Transform2d(shooterOffsetX, shooterOffsetY, Rotation2d.kZero))
            .getTranslation();

        // Iterative convergence — each iteration updates distance, which updates air time
        double distanceToTarget = target.getDistance(shooterPosition);
        Translation2d virtualTarget = target;

        for (int i = 0; i < 20; i++) {
            double airTime = ShotCalculatorConstants.getAirTime(distanceToTarget);

            // Offset the target to compensate for robot movement during flight
            virtualTarget = target.minus(new Translation2d(
                shooterVelocityX * airTime,
                shooterVelocityY * airTime));

            // Recalculate distance with the new virtual target
            distanceToTarget = virtualTarget.getDistance(shooterPosition);
        }

        lastTargetPosition = rawTarget.getTranslation();
        lastCompensatedDistance = distanceToTarget;
        Pose2d virtualTargetPose = new Pose2d(virtualTarget, Rotation2d.kZero);

        // Log
        Logger.recordOutput("ShotCalculator/VirtualTargetPose", virtualTargetPose);
        Logger.recordOutput("ShotCalculator/CompensatedDistance", distanceToTarget);
        Logger.recordOutput("ShotCalculator/ShooterVelocityX", shooterVelocityX);
        Logger.recordOutput("ShotCalculator/ShooterVelocityY", shooterVelocityY);

        return virtualTargetPose;
    }

    /**
     * Returns the compensated distance from the shooter to a target,
     * accounting for phase delay and velocity. Useful for shooter speed lookups.
     *
     * If the target matches the last getVirtualTarget() call, returns the cached value.
     * Otherwise, computes a fresh calculation.
     *
     * @param rawTarget The actual target position on the field
     * @return The compensated distance in meters
     */
    public double getCompensatedDistance(Pose2d rawTarget) {
        // If same target as last getVirtualTarget call, use cached value
        if (lastTargetPosition != null && lastTargetPosition.equals(rawTarget.getTranslation())) {
            return lastCompensatedDistance;
        }
        // Otherwise compute fresh
        getVirtualTarget(rawTarget);
        return lastCompensatedDistance;
    }
}