package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

    public boolean startedAutoAligning = false;
    public enum robotSideState {
        LEFT,
        RIGHT,
        IDLE
    }
    public robotSideState currentState = robotSideState.IDLE;
    private Constraints xConstraints = new Constraints(0.01, 1);
    private Constraints yConstraints = new Constraints(0.01, 1);
    private Constraints rotConstraints = new Constraints(2, 5);
    public ProfiledPIDController xController = new ProfiledPIDController(0.10, 0, 0, xConstraints);
    public ProfiledPIDController yController = new ProfiledPIDController(0.05, 0, 0, yConstraints);
    public ProfiledPIDController spinController = new ProfiledPIDController(0.04, 0, 0, rotConstraints);

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
            return xController.calculate(getRobotX(), wantedX) * flipped;
        } else {
            return 0;
        }
    }

    public double moveRobotPoseY() {
        if(getRobotY() != 0 && wantedY != 0) {
            return yController.calculate(getRobotY(), wantedY) * flipped;
        } else {
            return 0;
        }
    }
    public double moveRobotPoseSpin() {
        return spinController.calculate(getYaw(), wantedRot);
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
        SmartDashboard.putNumber("Vision PID Rot", moveRobotPoseSpin());

        SmartDashboard.putNumber("Current TAG", currentTag);
    }
    
    public void setRobotSide(robotSideState state) {
        currentState = state;
    }

    public robotSideState getState() {
        return currentState;
    }

    public boolean isAutoAligning() {
        return getYaw() >= (VisionConstants.TargetRot + 3) || getYaw() <= (VisionConstants.TargetRot - 3);
    }
    public void update() {
        log();
        switch (currentState) {
            case LEFT:
                if(!isAutoAligning()) {
                    startedAutoAligning = true;
                } 
                if(startedAutoAligning) {
                    wantedX = VisionConstants.TargetLeftXPos;
                    wantedY = VisionConstants.TargetLeftYPos;
                } else {
                    wantedX = 0;
                    wantedY = 0;
                }
                wantedRot = VisionConstants.TargetRot;
                flipped = -1;
                break;
            case RIGHT:
                if(!isAutoAligning()) {
                    startedAutoAligning = true;
                } 
                if(startedAutoAligning) {
                    wantedX = VisionConstants.TargetRightXPos;
                    wantedY = VisionConstants.TargetRightYPos;
                }
                else {
                    wantedX = 0;
                    wantedY = 0;
                }
                wantedRot = VisionConstants.TargetRot;
                flipped = -1;
                break;
            case IDLE:
                wantedX = 0;
                wantedY = 0;
                wantedRot = 0;
                flipped = -1;
                startedAutoAligning = false;
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
