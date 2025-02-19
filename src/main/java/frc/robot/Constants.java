package frc.robot;

import com.google.gson.FieldNamingStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int TIMEOUT_MS = 60;

     public static class VisionConstants {
        public static final String kCameraName = "PlasmaCam";
        private static final double camPitch = Units.degreesToRadians(30.0);
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 3;
        public static final int ROT_MOTOR_ID = 2;
        public static final int INDEX_MOTOR_ID = 7;

        public static final double INTAKE_SPEED = 0.5;
        public static final double INDEX_SPEED = 0.6;

        //Positions
        public static final double INTAKE_UP_POS = 0;
        public static final double INTAKE_DOWN_POS = 0;

        // Rotation Pid
        public static final double intakePivotKS = 0.25;
        public static final double intakePivotKV = 0.12;
        public static final double intakePivotKP = 10;
        public static final double intakePivotKD = 0;

        public static final double intakePivotVel = 180;
        public static final double intakePivotAccel = 360;
        public static final double intakePivotJerk = 3600;

        // Velocity Pid
        public static final double intakeVelocityKS = 0.25;
        public static final double intakeVelocityKV = 0.12;
        public static final double intakeVelocityKA = 0.01;
        public static final double intakeVelocityKP = 0.11;
        public static final double intakeVelocityKI = 0;
        public static final double intakeVelocityKD = 0;

        public static final double intakeVelocityAccel = 400;
        public static final double intakeVelocityJerk = 4000; 

    }

    public class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ONE_ID = 4;
        public static final int ELEVATOR_MOTOR_TWO_ID = 8;

        // Elevator Poses
        public static final double LEVEL1_HEIGHT = 70;
        public static final double LEVEL2_HEIGHT = 20;
        public static final double LEVEL3_HEIGHT = 40;
        public static final double LEVEL4_HEIGHT = 70;
        public static final double FEEDER_HEIGHT = 0;


        public static final double elevatorPosKS = 0.25;
        public static final double elevatorPosKV = 0.12;
        public static final double elevatorPosKP = 10;
        public static final double elevatorPosKD = 0;

        public static final double elevatorPosVel = 100;
        public static final double elevatorPosAccel = 200;
        public static final double elevatorPosJerk = 3600;

    }

    public class ArmConstants {
        public static final int armRotID = 5;
        public static final int armOuttakeID = 6;
        public static final int armCancoderID = 1;
        public static final double armRunSpeed = 0.3;

        public static final double armPosKS = 0.25;
        public static final double armPosKV = 0.3;
        public static final double armPosKA = 0.01;
        public static final double armPosKP = 5.0;
        public static final double armPosKD = 0;

        // Arm Poses
        public static final double armStowedPos = 0;
        public static final double armLowPos = 0;
        public static final double armMidPos = 0.5;
        public static final double armHighPos = 0;
        public static final double armFeedPos = 0;

        public static final double armPosVel = 180;
        public static final double armPosAccel = 360;
        public static final double armPosJerk = 2000;
        
    }
    public class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 0;
        public static final double CLIMB_SPEED = 0.5;
         
        public static final int PID_IDX = 1;

        // ----------- PID ------------ //
        public static final double kF = 0.1;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        
    }
}
