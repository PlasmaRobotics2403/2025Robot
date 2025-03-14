package frc.robot;

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

        /* - - - - - - - TARGET 6 POSES - - - - - - - - */
        public static final double Target6LeftXPos = 0.86; 
        public static final double Target6LeftYPos = -2.8; 
        public static final double Target6RightXPos = 0.87;
        public static final double Target6RightYPos = 0.37;

        /* - - - - - - - TARGET 7 POSES - - - - - - - - */
        public static final double Target7LeftXPos = 0.89; 
        public static final double Target7LeftYPos = -3.021; 
        public static final double Target7RightXPos = 0.82;
        public static final double Target7RightYPos = 0.17;

        /* - - - - - - - TARGET 8 POSES - - - - - - - - */
        public static final double Target8LeftXPos = 0.86; 
        public static final double Target8LeftYPos = -2.85; 
        public static final double Target8RightXPos = 0.87;
        public static final double Target8RightYPos = 0.12;

        /* - - - - - - - TARGET 9 POSES - - - - - - - - */
        public static final double Target9LeftXPos = 0.86; 
        public static final double Target9LeftYPos = -2.84; 
        public static final double Target9RightXPos = 0.86;
        public static final double Target9RightYPos = 0.65;

        /* - - - - - - - TARGET 10 POSES - - - - - - - - */
        public static final double Target10LeftXPos = 1.265; 
        public static final double Target10LeftYPos = -3.021; 
        public static final double Target10RightXPos = 1.434;
        public static final double Target10RightYPos = 0.27;

        /* - - - - - - - TARGET 11 POSES - - - - - - - - */
        public static final double Target11LeftXPos = 0.86; 
        public static final double Target11LeftYPos = -2.9; 
        public static final double Target11RightXPos = 0.87;
        public static final double Target11RightYPos = 0.27;

        /* - - - - - - - TARGET 17 POSES - - - - - - - - */
        
        public static final double Target17LeftXPos = 1.265; 
        public static final double Target17LeftYPos = -3.021; 
        public static final double Target17RightXPos = 1.434;
        public static final double Target17RightYPos = 0.27;

        /* - - - - - - - TARGET 18 POSES - - - - - - - - */
        public static final double Target18LeftXPos = 1.265; 
        public static final double Target18LeftYPos = -3.021; 
        public static final double Target18RightXPos = 1.434;
        public static final double Target18RightYPos = 0.27;

        /* - - - - - - - TARGET 19 POSES - - - - - - - - */
        public static final double Target19LeftXPos = 1.265; 
        public static final double Target19LeftYPos = -3.021; 
        public static final double Target19RightXPos = 1.434;
        public static final double Target19RightYPos = 0.27;

        /* - - - - - - - TARGET 20 POSES - - - - - - - - */
        public static final double Target20LeftXPos = 1.265; 
        public static final double Target20LeftYPos = -3.021; 
        public static final double Target20RightXPos = 1.434;
        public static final double Target20RightYPos = 0.27;

        /* - - - - - - - TARGET 21 POSES - - - - - - - - */
        public static final double Target21LeftXPos = 1.265; 
        public static final double Target21LeftYPos = -3.021; 
        public static final double Target21RightXPos = 1.434;
        public static final double Target21RightYPos = 0.27;

        /* - - - - - - - TARGET 22 POSES - - - - - - - - */
        public static final double Target22LeftXPos = 1.265; 
        public static final double Target22LeftYPos = -3.021; 
        public static final double Target22RightXPos = 1.434;
        public static final double Target22RightYPos = 0.27;
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 12;
        public static final int ROT_MOTOR_ID = 13;
        public static final int INDEX_MOTOR_ID = 7;

        public static final double INTAKE_SPEED = 0.6;
        public static final double INDEX_SPEED = 0.6;

        public static final double INTAKE_WAIT_TIME = 0.01;

        //Positions
        public static final double INTAKE_UP_POS = 0;
        public static final double INTAKE_DOWN_POS = 20.3;

        // Rotation Pid
        public static final double intakePivotKS = 0.25;
        public static final double intakePivotKV = 0.12;
        public static final double intakePivotKP = 0.2;
        public static final double intakePivotKD = 0;

        public static final double intakePivotVel = 60;
        public static final double intakePivotAccel = 180;
        public static final double intakePivotJerk = 3000;

        // Velocity Pid
        public static final double intakeVelocityKS = 0.25;
        public static final double intakeVelocityKV = 0.12;
        public static final double intakeVelocityKA = 0.01;
        public static final double intakeVelocityKP = 0.2;
        public static final double intakeVelocityKI = 0;
        public static final double intakeVelocityKD = 0;

        public static final double intakeVelocityAccel = 400;
        public static final double intakeVelocityJerk = 2000; 

    }

    public class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ONE_ID = 4;
        public static final int ELEVATOR_MOTOR_TWO_ID = 8;
        public static final int ELEVATOR_LIMIT_SWITCH_ID = 1;

        // Elevator Poses
        public static final double ARM_THRESHHOLD = 24.2;
        public static final double LEVEL1_HEIGHT = 34;
        public static final double LEVEL2_HEIGHT = 34;
        public static final double LEVEL3_HEIGHT = 47;
        public static final double LEVEL4_HEIGHT = 110;
        public static final double FEEDER_HEIGHT = 50;


        public static final double elevatorPosKS = 0.25;
        public static final double elevatorPosKV = 0.12;
        public static final double elevatorPosKP = 6;
        public static final double elevatorPosKD = 0;

        public static final double elevatorPosVel = 100;
        public static final double elevatorPosAccel = 300;
        public static final double elevatorPosJerk = 3600;

    }

    public class ArmConstants {
        public static final int armRotID = 5;
        public static final int armOuttakeID = 6;
        public static final int armCancoderID = 1;
        public static final double armRunSpeed = 0.2;
        public static final double armGearRatio = 52.820625;
        public static final double armGravityK = 1;

        public static final double armPos0KS = 0.25;
        public static final double armPos0KV = 0.12;
        public static final double armPos0KA = 0.1;
        public static final double armPos0KP = 28;
        public static final double armPos0KD = 0.1;


        public static final double armPos1KS = 0.25;
        public static final double armPos1KV = 0.12;
        public static final double armPos1KA = 0.1;
        public static final double armPos1KP = 25;
        public static final double armPos1KD = 0;

        // Arm Poses
        public static final double armStowedPos = 0.11;
        public static final double armLowPos = 0.12;
        public static final double armMidPos = -0.5;
        public static final double armHighPos = -0.5;
        public static final double armFeedPos = 0.1;

        public static final double armPosVel = 0.08;
        public static final double armPosAccel = 0.5;
        public static final double armPosJerk = 1600;
        
    }
    public class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 0;
        public static final double CLIMB_SPEED = 0.2;
         
        public static final int PID_IDX = 1;

        // ----------- PID ------------ //
        public static final double kF = 0.1;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        
    }
}
