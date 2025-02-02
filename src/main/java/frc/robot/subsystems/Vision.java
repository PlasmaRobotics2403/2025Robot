package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {

    public enum robotSideState {
        LEFT,
        RIGHT
    }
    public robotSideState currenState = robotSideState.LEFT;
    public PIDController movementXController;
    public PIDController movementYController;

    private Transform3d robotTransform3d = new Transform3d();
    private Pose3d robotPose = new Pose3d();
    PhotonCamera camera;
    public Vision() {

        camera = new PhotonCamera("Plasma Cam");
        if (camera.getLatestResult() != null) {
        
            var result = camera.getLatestResult();
        
            if(result.getBestTarget() != null){
                PhotonTrackedTarget target = result.getBestTarget();
                AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotTransform3d);
                }
            }
        }
  
    }
    public double moveRobotPoseLeft() {
        if(camera.getAllUnreadResults() != null) {
            return 0; 
        }
        return 0;
    }
    public void log() {
        SmartDashboard.putNumber("RobotPoseX", robotPose.getX());
        SmartDashboard.putNumber("RobotPoseY", robotPose.getY());
        SmartDashboard.putNumber("Robot Yaw", robotPose.getRotation().getZ());
        SmartDashboard.putData(null);
    }
    
    public void setRobotSide(robotSideState state) {
        currenState = state;
    }

    public robotSideState getState() {
        return currenState;
    }

    
    public void update() {
        log();

        switch (currenState) {
            case LEFT:

                break;
        
            case RIGHT:
                
                break;
        }
        if (camera.getLatestResult() != null) {
        
            var result = camera.getLatestResult();
        
            if(result.getBestTarget() != null){
                PhotonTrackedTarget target = result.getBestTarget();
                AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotTransform3d);
                }
            }
        }
    }
}
