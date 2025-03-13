package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;

public class Vision extends SubsystemBase {

    public Vision(){
        for (String limelightName : LIMELIGHT_NAMES){
            // Switch to pipeline 0, change how Limelight processes images
            LimelightHelpers.setPipelineIndex(limelightName, 0);

            /* AprilTag functionality */
            // Set a custom crop window for improved performance (-1 to 1 for each value)
            LimelightHelpers.setCropWindow(limelightName, -0.5, 0.5, -0.5, 0.5);

            // Set AprilTag offset tracking point (meters)
            LimelightHelpers.setFiducial3DOffset(
                limelightName, 
                0.0,    // Forward offset
                0.0,    // Side offset  
                0.5     // Height offset
            );

            // Set robot orientation
            LimelightHelpers.SetRobotOrientation(
                limelightName, 
                0.0, // m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                0.0, 
                0.0, 
                0.0, 
                0.0, 
                0.0
            );

            // Configure AprilTag detection
            // Only track these tag IDs
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{1, 2, 3, 4});
            // Process at half resolution for improved framerate and reduced range
            LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f);

            // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
            // todo different per camera?
            LimelightHelpers.setCameraPose_RobotSpace(
                limelightName, 
                0.5,    // Forward offset (meters)
                0.0,    // Side offset (meters)
                0.5,    // Height offset (meters)
                0.0,    // Roll (degrees)
                30.0,   // Pitch (degrees)
                0.0     // Yaw (degrees)
            );

        }
    }


    @Override
    public void periodic(){
        for (String limelightName : LIMELIGHT_NAMES){
            LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
            if (results.valid) {
                // AprilTags/Fiducials
                if (results.targets_Fiducials.length > 0) {
                    LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                    double id = tag.fiducialID;          // Tag ID
                    String family = tag.fiducialFamily;   // Tag family (e.g., "16h5")
                    
                    // 3D Pose Data
                    Pose3d robotPoseField = tag.getRobotPose_FieldSpace();    // Robot's pose in field space
                    Pose3d cameraPoseTag = tag.getCameraPose_TargetSpace();   // Camera's pose relative to tag
                    Pose3d robotPoseTag = tag.getRobotPose_TargetSpace();     // Robot's pose relative to tag
                    Pose3d tagPoseCamera = tag.getTargetPose_CameraSpace();   // Tag's pose relative to camera
                    Pose3d tagPoseRobot = tag.getTargetPose_RobotSpace();     // Tag's pose relative to robot
                    
                    // 2D targeting data
                    double tx = tag.tx;                  // Horizontal offset from crosshair
                    double ty = tag.ty;                  // Vertical offset from crosshair
                    double ta = tag.ta;                  // Target area (0-100% of image)

                    // Update SmartDashboard
                    SmartDashboard.putNumber(limelightName + "_" + id + " tx", tx);
                    SmartDashboard.putNumber(limelightName + "_" + id + " ty", ty);
                    SmartDashboard.putNumber(limelightName + "_" + id + " ta", ta);
                }                
            }
        }
    }
}
