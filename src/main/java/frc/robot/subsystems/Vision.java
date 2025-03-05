package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class Vision {

    public PhotonTrackedTarget lastTarget = new PhotonTrackedTarget();
    private double wantedX = 0;
    private double wantedY = 0;
    private double wantedRot = 0;
    private int currentTag = 0;
    private double flipped = 1;
    public Pigeon2 pigeon2;
    public enum robotSideState {
        LEFT,
        RIGHT,
        IDLE
    }
    public robotSideState currentState = robotSideState.IDLE;
    public PIDController movementXController = new PIDController(0.2, 0, 0);
    public PIDController movementYController = new PIDController(0.1, 0, 0);
    public PIDController movementSpinController = new PIDController(1, 0, 0);
    private Rotation3d rotation = new Rotation3d();

    private Transform3d robotTransform3d = new Transform3d(0, 0, 0, rotation);
    private Pose3d robotPose = new Pose3d(0, 0, 0, rotation);
    PhotonCamera camera;
    public Vision() {
        //movementXController = new PIDController(0.3, 0, 0);
        //movementYController = new PIDController(0.3, 0, 0);
       pigeon2 = new Pigeon2(0, "swerve");
       pigeon2.reset();
        camera = new PhotonCamera("Plasma Cam");
        if (camera.getLatestResult() != null) {
        
            var result = camera.getLatestResult();
        
            if(result.getBestTarget() != null){
                PhotonTrackedTarget target = result.getBestTarget();
                lastTarget = target;
                AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {

                    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotTransform3d);
                }
            }
        }
  
    }

    public double moveRobotPoseX() {
        if(getRobotX() != 0 && wantedX != 0) {
            return movementXController.calculate(getRobotX(), wantedX) * flipped;
        } else {
            return 0;
        }
    }

    public double moveRobotPoseY() {
        if(getRobotY() != 0 && wantedY != 0) {
            return movementYController.calculate(getRobotY(), wantedY) * flipped;
        } else {
            return 0;
        }
    }
    public double moveRobotPoseSpin() {
        return movementSpinController.calculate(getYaw(), wantedRot);
    }
    public double getRobotX() {
        try {
            return lastTarget.getBestCameraToTarget().getX()*100;
        } catch(Exception e) {
            return 0;
        }

    }
    public double getRobotY() {
        try {
            return lastTarget.getBestCameraToTarget().getY()*10;
        } catch(Exception e) {
            return 0;
        }

    }
    public double getYaw() {
        return pigeon2.getYaw().getValueAsDouble();
    }
    public double getWantedYaw() {
        return wantedRot;
    }
    
    public void log() {
        SmartDashboard.putNumber("RobotPoseX", getRobotX());
        SmartDashboard.putNumber("RobotPoseY", getRobotY());
        SmartDashboard.putNumber("Robot Yaw", getYaw());
        SmartDashboard.putNumber("Wanted X", wantedX);
        SmartDashboard.putNumber("Wanted Y", wantedY);
        SmartDashboard.putNumber("Vision PID X", moveRobotPoseX());
        SmartDashboard.putNumber("Vision PID Y", moveRobotPoseY());
        SmartDashboard.putNumber("Vision PID Rot", movementSpinController.getError());

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
                wantedRot = VisionConstants.TargetRot;
                flipped = -1;
                break;
            case RIGHT:
                wantedX = VisionConstants.TargetRightXPos;
                wantedY = VisionConstants.TargetRightYPos;
                wantedRot = VisionConstants.TargetRot;
                flipped = -1;
                break;
            case IDLE:
                wantedX = 0;
                wantedY = 0;
                wantedRot = 0;
                flipped = -1;
                break;
        }
        if (camera.getLatestResult() != null) {
        
            var result = camera.getLatestResult();
            if(result.getBestTarget() != null) {
                currentTag = result.getBestTarget().getFiducialId();
                PhotonTrackedTarget target = result.getBestTarget();
                lastTarget = target;
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
