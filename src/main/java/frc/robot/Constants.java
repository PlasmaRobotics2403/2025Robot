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
