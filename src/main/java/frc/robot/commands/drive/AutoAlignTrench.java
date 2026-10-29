// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.annotation.Target;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.PassTargeting;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignTrench extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
    private final XboxController m_driverController;

    private Pose2d targetPosition;

    private ShuffleboardTab driverTab;

    private Pigeon2 m_Pigeon2;

    // NetworkTables publishers so AdvantageScope can show the aim LIVE (sim or on the
    // robot). SignalLogger only writes to the hoot log, which AdvantageScope can't
    // stream live. Created once here so the topics exist as soon as the robot boots.
    private final StructPublisher<Pose2d> m_targetPub = NetworkTableInstance.getDefault()
            .getStructTopic("Pass/TargetPoint", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> m_desiredPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("Pass/DesiredRobotPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> m_flywheelRefPub = NetworkTableInstance.getDefault()
            .getStructTopic("Pass/FlywheelRefPoint", Pose2d.struct).publish();

    // Field-centric request: driver controls X/Y, heading PID supplies rotation
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  /** Creates a new AutoAlignTrench. */
  public AutoAlignTrench(CommandSwerveDrivetrain drivetrain, XboxController driverController) {
    m_drivetrain = drivetrain;

        m_driverController = driverController;

        driverTab = Shuffleboard.getTab("Driver");
        m_Pigeon2 = new Pigeon2(Constants.DriveConstants.GYRO_ID);

        addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // New press: forget the last pass target so we pick fresh.
    PassTargeting.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: It should also align with the trench in position, not just rotation.

    // Re-pick the target every loop (with hysteresis inside PassTargeting) so the
    // aim follows the robot as it moves. Shooter.distanceFromTrench() reads the
    // same point, so flywheel speed and heading always agree.
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Translation2d robot = m_drivetrain.getPose().getTranslation();
    targetPosition = PassTargeting.update(isBlue, robot);

    double headingCorrection = m_drivetrain.getHeadingPIDOutput(targetPosition);

    m_drivetrain.setControl(driveRequest
            .withVelocityX(m_driverController.getLeftY() * MaxSpeed)
            .withVelocityY(m_driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(headingCorrection));

    // ---- Hoot logging (Phoenix SignalLogger): where are we trying to aim? ----
    // Only runs while this command is scheduled, i.e. while a pass is held.
    // In AdvantageScope's 2D field, drag the two Pose2d signals onto the field.

    // The pass point being passed toward (shows as a marker on the field).
    SignalLogger.writeStruct("Pass/TargetPoint", Pose2d.struct, targetPosition);

    // A "ghost robot" at the robot's position, rotated to the heading the PID is
    // driving toward. The robot's BACK should face the target, so this ghost's
    // front points directly away from the target.
    Rotation2d aimHeading = m_drivetrain.getTargetHeading(targetPosition.getTranslation());
    Pose2d desiredPose = new Pose2d(robot, aimHeading);
    SignalLogger.writeStruct("Pass/DesiredRobotPose", Pose2d.struct, desiredPose);

    // The trench point the FLYWHEEL distance is measured to (same side as the aim point).
    Pose2d flywheelRef = PassTargeting.getFlywheelReference(isBlue, robot);
    SignalLogger.writeStruct("Pass/FlywheelRefPoint", Pose2d.struct, flywheelRef);

    // The same poses over NetworkTables for live viewing in AdvantageScope.
    m_targetPub.set(targetPosition);
    m_desiredPosePub.set(desiredPose);
    m_flywheelRefPub.set(flywheelRef);

    // Signed heading error (desired minus actual), in degrees. Should settle near 0.
    SignalLogger.writeDouble("Pass/HeadingErrorDeg",
            aimHeading.minus(m_drivetrain.getPose().getRotation()).getDegrees(), "degrees");

    // True while a pass is being held; makes presses easy to find in the log.
    SignalLogger.writeBoolean("Pass/Active", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SignalLogger.writeBoolean("Pass/Active", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
