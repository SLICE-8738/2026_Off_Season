// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.annotation.Target;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignTrench extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
    private final XboxController m_driverController;

    private Pose2d targetPosition;

    private ShuffleboardTab driverTab;

    private Pigeon2 m_Pigeon2;

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
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    // get distance to each trench
    if (isBlue){
      Pose2d leftTarget = Constants.AlignTargets.getVirtualRed_TrenchLeft();
      Pose2d rightTarget = Constants.AlignTargets.getVirtualBlue_TrenchRight();
      double leftDistance = m_drivetrain.getDistanceTo(leftTarget);
      double rightDistance = m_drivetrain.getDistanceTo(rightTarget);
      targetPosition = (leftDistance < rightDistance) ? leftTarget : rightTarget;
    } else {
      Pose2d leftTarget = Constants.AlignTargets.getVirtualRed_TrenchLeft();
      Pose2d rightTarget = Constants.AlignTargets.getVirtualBlue_TrenchRight();
      double leftDistance = m_drivetrain.getDistanceTo(leftTarget);
      double rightDistance = m_drivetrain.getDistanceTo(rightTarget);
      targetPosition = (leftDistance < rightDistance) ? leftTarget : rightTarget;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: It should also align with the trench in position, not just rotation.
    double headingCorrection = m_drivetrain.getHeadingPIDOutput(targetPosition);
        
    m_drivetrain.setControl(driveRequest
            .withVelocityX(m_driverController.getLeftY() * MaxSpeed)
            .withVelocityY(m_driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(headingCorrection));
    
    
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
