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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class Vision {

    private double wantedX = 0;
    private double wantedY = 0;
    private int currentTag = 0;
    private double flipped = 1;
    public enum robotSideState {
        LEFT,
        RIGHT,
        IDLE
    }
    public robotSideState currentState = robotSideState.IDLE;
    public PIDController movementXController = new PIDController(1, 0, 0);
    public PIDController movementYController = new PIDController(1, 0, 0);
    private Rotation3d rotation = new Rotation3d();

    private Transform3d robotTransform3d = new Transform3d();
    private Pose3d robotPose = new Pose3d(0, 0, 0, rotation);
    PhotonCamera camera;
    public Vision() {
        //movementXController = new PIDController(0.3, 0, 0);
        //movementYController = new PIDController(0.3, 0, 0);
        DriverStation.reportWarning("INITIALLIZE!!!!!!!!!", false);
       
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

    public double moveRobotPoseX() {
        double currentX = robotPose.getX();
        if(robotPose.getX() != 0) {
            return movementXController.calculate(currentX, wantedX) * flipped;
        } else {
            return 0;
        }
    }

    public double moveRobotPoseY() {
        if(robotPose.getY() != 0) {
            return movementYController.calculate(robotPose.getY(), wantedY) * flipped;
        } else {
            return 0;
        }
    }
    public void log() {
        SmartDashboard.putNumber("RobotPoseX", robotPose.getX());
        SmartDashboard.putNumber("RobotPoseY", robotPose.getY());
        SmartDashboard.putNumber("Robot Yaw", robotPose.getRotation().getZ());
        SmartDashboard.putNumber("Wanted X", wantedX);
        SmartDashboard.putNumber("Wanted Y", wantedY);
        SmartDashboard.putNumber("Vision PID X", moveRobotPoseX());
        SmartDashboard.putNumber("Vision PID Y", moveRobotPoseY());

        SmartDashboard.putNumber("Current TAG", currentTag);
    }
    
    public void setRobotSide(robotSideState state) {
        currentState = state;
    }

    public robotSideState getState() {
        return currentState;
    }

    
    public void update() {
        log();

        switch (currentState) {
            case LEFT:
                wantedX = VisionConstants.TargetLeftXPos;
                wantedY = VisionConstants.TargetLeftYPos;
                flipped = -1;
                break;
            case RIGHT:
                wantedX = VisionConstants.TargetRightXPos;
                wantedY = VisionConstants.TargetRightYPos;
                flipped = -1;
                break;
            case IDLE:
                wantedX = 0;
                wantedY = 0;
                flipped = -1;
                break;
        }
        if (camera.getLatestResult() != null) {
        
            var result = camera.getLatestResult();
            if(result.getBestTarget() != null) {
                currentTag = result.getBestTarget().getFiducialId();
                PhotonTrackedTarget target = result.getBestTarget();
                AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotTransform3d);
                }
            } else {
                currentTag = 0;
            }
        }
    }
}
