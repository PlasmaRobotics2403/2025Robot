package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

    public PIDController xController = new PIDController(0.14, 0, 0, 0.02); // 0.16
    public PIDController yController = new PIDController(0.08, 0, 0.001, 0.02);// 0.166
    public PIDController spinController = new PIDController(0.03, 0, 0.001, 0.02);// 0.05

    private Rotation3d rotation = new Rotation3d();
    
    private Transform3d robotTransform3d = new Transform3d(0, 0, 0, rotation);
    private Pose3d robotPose = new Pose3d(0, 0, 0, rotation);
    PhotonCamera camera;
    public Vision() {
        //movementXController = new PIDController(0.3, 0, 0);
        //movementYController = new PIDController(0.3, 0, 0);
        spinController.enableContinuousInput(-180, 180);
        xController.setTolerance(0.5, 0.1);
        yController.setTolerance(0.5, 0.1);

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
        if(getRobotX() != 0 && wantedX != 0 && currentTag != 0) {
            return xController.calculate(getRobotX(), wantedX) * flipped;
        } else {
            return 0;
        }
    }

    public double moveRobotPoseY() {
        if(getRobotY() != 0 && wantedY != 0 && currentTag != 0) {
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
        if(pigeon2.getYaw().getValueAsDouble()>0.01) {
            return ((pigeon2.getYaw().getValueAsDouble()+180)%360)-180;
        } else {
            return ((pigeon2.getYaw().getValueAsDouble()-180)%360)+180;
        }
    }
    public double getWantedYaw() {
        return wantedRot;
    }
    public int getApriltagNumber() {
        return currentTag;
    }
    public void log() {
        SmartDashboard.putNumber("RobotPoseX", getRobotX());
        SmartDashboard.putNumber("RobotPoseY", getRobotY());
        SmartDashboard.putNumber("Robot Yaw", getYaw());

        SmartDashboard.putNumber("Wanted Rot", wantedRot);
        SmartDashboard.putNumber("Wanted X", wantedX);
        SmartDashboard.putNumber("Wanted Y", wantedY);
        SmartDashboard.putNumber("Vision PID X", moveRobotPoseX());
        SmartDashboard.putNumber("Vision PID Y", moveRobotPoseY());
        SmartDashboard.putNumber("Vision PID Rot", moveRobotPoseSpin());

        SmartDashboard.putBoolean("Has Started Auto Aligning", startedAutoAligning);
        SmartDashboard.putBoolean("Is Auto Rotating", isAutoRotating());

        SmartDashboard.putBoolean("X At Setpoint", xController.atSetpoint());
        SmartDashboard.putBoolean("Y At Setpoint", yController.atSetpoint());


        SmartDashboard.putNumber("Current TAG", getApriltagNumber());
    }
    
    public void setRobotSide(robotSideState state) {
        currentState = state;
    }

    public robotSideState getState() {
        return currentState;
    }

    public boolean isAutoRotating() {
        return getYaw() >= (wantedRot + 2) || getYaw() <= (wantedRot - 2);
    }
    public void update() {
        log();
        switch (currentState) {
            case LEFT:
                if(!isAutoRotating()) {
                    startedAutoAligning = true;
                } 

                switch(getApriltagNumber()) {
                    case 6:
                        wantedRot = 120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 7:
                        wantedRot = 180;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 8:
                        wantedRot = -120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 9:
                        wantedRot = -60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 10:
                        wantedRot = 0;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 11:
                        wantedRot = 60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 17:
                        wantedRot = -120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 18:
                        wantedRot = 180;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 19:
                        wantedRot = 120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 20:
                        wantedRot = 60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 21:
                        wantedRot = 0;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 22:
                        wantedRot = -60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6LeftXPos;
                            wantedY = VisionConstants.Target6LeftYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;

                }
                flipped = 1;
                break;
            case RIGHT:
                if(!isAutoRotating()) {
                    startedAutoAligning = true;
                } 
                switch(getApriltagNumber()) {
                    case 6:
                        wantedRot = 120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 7:
                        wantedRot = 180;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 8:
                        wantedRot = -120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 9:
                        wantedRot = -60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 10:
                        wantedRot = 0;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 11:
                        wantedRot = 60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 17:
                        wantedRot = -120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 18:
                        wantedRot = 180;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 19:
                        wantedRot = 120;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 20:
                        wantedRot = 60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 21:
                        wantedRot = 0;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                    case 22:
                        wantedRot = -60;
                        if(startedAutoAligning) {
                            wantedX = VisionConstants.Target6RightXPos;
                            wantedY = VisionConstants.Target6RightYPos;
                        } else {
                            wantedX = 0;
                            wantedY = 0;
                        }
                        break;
                }
                flipped = 1;
                break;
            case IDLE:
                wantedX = 0;
                wantedY = 0;
                //wantedRot = getYaw();

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
