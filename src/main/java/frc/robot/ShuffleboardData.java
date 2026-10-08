package frc.robot;

import java.util.Map;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShuffleboardData extends SubsystemBase {

    CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    Indexer m_Indexer;
    Intake m_Intake;
    Shooter m_Shooter;
    
    ShuffleboardTab driverTab, autoTab, debugTab, shooterTuning, limelightTab;

    ///////////////////////////
    /// Driver Tab Values /////
    ///////////////////////////
    
    /* Driverstation Widgets */
    SimpleWidget isEnabled;
    SimpleWidget isBrownedOut;
    SimpleWidget batteryVoltage;
    
    /*Drivetrain Widgets */
    SimpleWidget drivetrainVelocityX;
    SimpleWidget drivetrainVelocityY;


    ///////////////////////////
    /// Debug Tab Values //////
    ///////////////////////////
    
    /* Drivetrain Widgets */

    /* Intake Widgets */
    SimpleWidget intakeAtStow;
    SimpleWidget intakeAtDeploy;
    SimpleWidget intakeExtendPosition;
    SimpleWidget intakeRunning;
    SimpleWidget intakeExtenderSpeed;
    SimpleWidget intakeRollerSpeed;
    SimpleWidget intakeRollerTargetSpeed;

    /* Indexer */
    SimpleWidget stageOneSpeed;
    SimpleWidget stageTwoSpeed;

    ///////////////////////////
    /// Debug Tab Values //////
    ///////////////////////////

    /* Shooter Widgets */
    SimpleWidget shooterRPM;

    ///////////////////////////
    /// Limelight Debugging ///
    ///////////////////////////

    /* Which cameras get a row on the tab. Names come from the drivetrain so they can't drift apart. */
    private static final String[] LIMELIGHT_NAMES = {
        CommandSwerveDrivetrain.LL_TRENCH,
        CommandSwerveDrivetrain.LL_HUB
    };

    /* Map for this tab only (separate from the Driver tab's field):
     * robot icon = the FUSED pose the drivetrain believes,
     * one extra "ghost" per camera = that camera's RAW pose. A ghost clipping into a wall = bad camera. */
    private final Field2d visionField = new Field2d();

    /* All the widgets that belong to ONE camera's row. */
    private static final class LimelightWidgets {
        String name;
        SimpleWidget fuseSwitch;   // ON = camera corrects the robot pose, OFF = shown but ignored (dashboard writes this)
        SimpleWidget status;       // "Fusing", "No tags", "Switched off", ...
        SimpleWidget tagCount;     // AprilTags in the latest estimate
        SimpleWidget avgTagDist;   // average tag distance, meters
        SimpleWidget ambiguity;    // worst tag ambiguity, 0..1 (-1 = no data)
        SimpleWidget poseX;        // raw camera pose X, meters
        SimpleWidget poseY;        // raw camera pose Y, meters

    }

    private final LimelightWidgets[] limelightWidgets = new LimelightWidgets[LIMELIGHT_NAMES.length];


    public ShuffleboardData(CommandSwerveDrivetrain m_CommandSwerveDrivetrain, Intake m_Intake, Indexer m_Indexer ,Shooter m_Shooter){
        
        this.m_CommandSwerveDrivetrain = m_CommandSwerveDrivetrain;
        this.m_Indexer = m_Indexer;
        this.m_Intake = m_Intake;
        this.m_Shooter = m_Shooter;

        driverTab = Shuffleboard.getTab("Driver");
        autoTab = Shuffleboard.getTab("Autonomous");
        debugTab = Shuffleboard.getTab("Debug");
        shooterTuning = Shuffleboard.getTab("Shooter Tuning");
        limelightTab = Shuffleboard.getTab("Limelight Debugging");

        ///////////////////////////
        /// Driver Tab Values /////
        ///////////////////////////
        
        
        
        /* Driverstation */
        isEnabled = driverTab.add("Enabled", DriverStation.isEnabled())
            .withWidget(BuiltInWidgets.kBooleanBox);
        isBrownedOut = driverTab.add("Browned Out", RobotController.isBrownedOut())
            .withWidget(BuiltInWidgets.kBooleanBox);
        batteryVoltage = driverTab.add("Battery Voltage", RobotController.getBatteryVoltage());
        
        /* Drivetrain */
        drivetrainVelocityX = driverTab.add("Drivetrain Velocity X", m_CommandSwerveDrivetrain.getChassisSpeeds().vxMetersPerSecond);
        drivetrainVelocityY = driverTab.add("Drivetrian Velocity Y", m_CommandSwerveDrivetrain.getChassisSpeeds().vyMetersPerSecond);

        
        ///////////////////////////
        /// Debug Tab Values //////
        ///////////////////////////

        /* Intake */
        intakeAtStow = debugTab.add("Intake At Stow", m_Intake.isStowed())
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeAtDeploy = debugTab.add("Intake At Deploy", m_Intake.isDeployed())
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeExtendPosition = debugTab.add("Intake Extension Position", m_Intake.getExtenderPosition());
        intakeRunning = debugTab.add("Intake Runs Command", m_Intake.getCurrentCommand() != null)
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeExtenderSpeed = debugTab.add("Intake Extender Speed", m_Intake.getVelocity()[0]);
        intakeRollerSpeed = debugTab.add("Intake Roller Speed", m_Intake.getRollerVelocity());
        intakeRollerTargetSpeed = debugTab.add("Intake Roller Target Speed", m_Intake.getRollerTargetSpeed());
        
        /* Indexer */
        stageOneSpeed = debugTab.add("Indexer Floor Speed", m_Indexer.getStageOneSpeed());
        stageTwoSpeed = debugTab.add("Indexer Roller Speed", m_Indexer.getStageTwoSpeed());
        

        ///////////////////////////
        /// Shooter Tab Values ////
        ///////////////////////////

        shooterRPM = shooterTuning.add("Shooter RPM", m_Shooter.getFlywheelSpeed());

        ///////////////////////////
        /// Limelight Debugging ///
        ///////////////////////////

        // The map takes the top-left 6x3 tiles of the tab.
        limelightTab.add("Vision Field", visionField)
            .withWidget(BuiltInWidgets.kField)
            .withPosition(0, 0)
            .withSize(6, 3);

        // One row of small tiles per camera, stacked underneath the map.
        for (int i = 0; i < LIMELIGHT_NAMES.length; i++) {
            LimelightWidgets w = new LimelightWidgets();
            String name = LIMELIGHT_NAMES[i];
            int row = 3 + i;
            w.name = name;

            // Defaults to ON so behavior matches the old code until someone flips it.
            w.fuseSwitch = limelightTab.add(name + " Fuse", true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(0, row).withSize(2, 1);
            w.status = limelightTab.add(name + " Status", "Starting")
                .withPosition(2, row).withSize(2, 1);
            w.tagCount = limelightTab.add(name + " Tags", 0.0)
                .withPosition(4, row).withSize(1, 1);
            w.avgTagDist = limelightTab.add(name + " Avg Dist (m)", 0.0)
                .withPosition(5, row).withSize(1, 1);
            w.ambiguity = limelightTab.add(name + " Ambiguity", -1.0)
                .withPosition(6, row).withSize(1, 1);
            w.poseX = limelightTab.add(name + " Pose X (m)", 0.0)
                .withPosition(7, row).withSize(1, 1);
            w.poseY = limelightTab.add(name + " Pose Y (m)", 0.0)
                .withPosition(8, row).withSize(1, 1);

            limelightWidgets[i] = w;
        }

        ///////////////////////
        /// Autonomous Tab ////
        ///////////////////////

    }

    @Override
    public void periodic(){

        ///////////////////
        /// Driver Tab ////
        ///////////////////
        
        isEnabled.getEntry().setBoolean(DriverStation.isEnabled());
        isBrownedOut.getEntry().setBoolean(RobotController.isBrownedOut());
        batteryVoltage.getEntry().setDouble(RobotController.getBatteryVoltage());
        
        drivetrainVelocityX.getEntry().setDouble(m_CommandSwerveDrivetrain.getChassisSpeeds().vxMetersPerSecond);
        drivetrainVelocityY.getEntry().setDouble(m_CommandSwerveDrivetrain.getChassisSpeeds().vyMetersPerSecond);

        ///////////////////////////
        /// Debug Tab Values //////
        ///////////////////////////

        /* Intake */
        intakeAtStow.getEntry().setBoolean(m_Intake.isStowed());
        intakeAtDeploy.getEntry().setBoolean(m_Intake.isDeployed());
        intakeExtendPosition.getEntry().setDouble(m_Intake.getExtenderPosition());
        intakeRunning.getEntry().setBoolean(m_Intake.getCurrentCommand() != null);
        intakeExtenderSpeed.getEntry().setDouble(m_Intake.getVelocity()[0]);
        intakeRollerSpeed.getEntry().setDouble(m_Intake.getRollerVelocity());
        intakeRollerTargetSpeed.getEntry().setDouble(m_Intake.getRollerTargetSpeed());

        /* Indexer */
        stageOneSpeed.getEntry().setDouble(m_Indexer.getStageOneSpeed());
        stageTwoSpeed.getEntry().setDouble(m_Indexer.getStageTwoSpeed());

        ///////////////////////////
        /// Shooter Tab Values ////
        ///////////////////////////

        shooterRPM.getEntry().setDouble(m_Shooter.getFlywheelSpeed());

        ///////////////////////////
        /// Limelight Debugging ///
        ///////////////////////////

        // Robot icon on this tab's map = the pose the drivetrain currently believes (fused).
        visionField.setRobotPose(m_CommandSwerveDrivetrain.getPose());

        for (LimelightWidgets w : limelightWidgets) {
            // Dashboard -> robot: hand this camera's Fuse switch to the drivetrain.
            m_CommandSwerveDrivetrain.setVisionFuseEnabled(w.name, w.fuseSwitch.getEntry().getBoolean(true));

            // Robot -> dashboard: show what the camera and drivetrain are doing.
            w.status.getEntry().setString(m_CommandSwerveDrivetrain.getVisionStatus(w.name));

            LimelightHelpers.PoseEstimate estimate = m_CommandSwerveDrivetrain.getLatestVisionEstimate(w.name);

            if (estimate == null || estimate.tagCount == 0) {
                // Camera sees nothing. NaN makes "no data" obvious instead of looking like a real 0.
                w.tagCount.getEntry().setDouble(0);
                w.avgTagDist.getEntry().setDouble(Double.NaN);
                w.ambiguity.getEntry().setDouble(-1);
                w.poseX.getEntry().setDouble(Double.NaN);
                w.poseY.getEntry().setDouble(Double.NaN);
                // setPoses() with no arguments removes this camera's ghost from the map.
                visionField.getObject(w.name).setPoses();
            } else {
                // Ambiguity is per tag (high = the solver can't tell two mirrored poses apart).
                // Show the WORST one across the tags used.
                double worstAmbiguity = -1;
                if (estimate.rawFiducials != null) {
                    for (LimelightHelpers.RawFiducial fiducial : estimate.rawFiducials) {
                        worstAmbiguity = Math.max(worstAmbiguity, fiducial.ambiguity);
                    }
                }

                w.tagCount.getEntry().setDouble(estimate.tagCount);
                w.avgTagDist.getEntry().setDouble(estimate.avgTagDist);
                w.ambiguity.getEntry().setDouble(worstAmbiguity);
                w.poseX.getEntry().setDouble(estimate.pose.getX());
                w.poseY.getEntry().setDouble(estimate.pose.getY());

                // This camera's raw pose as its own ghost robot on the map.
                visionField.getObject(w.name).setPose(estimate.pose);
            }
        }

    }


}
