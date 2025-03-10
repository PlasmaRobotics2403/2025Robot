package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private StateManager stateManager;
    private Swerve swerve;
    private Vision vision;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    public AutoRoutines(AutoFactory factory, StateManager stateManager, Swerve swerve, Vision vision) {
        m_factory = factory;
        this.stateManager = stateManager;
        this.swerve = swerve;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("TestAuto");
    
        routine.active().onTrue(
            simplePath.resetOdometry()
                 .andThen(simplePath.cmd())
        );
        
        return routine;
    }

    public AutoRoutine autoAlign() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("TestAuto");
    
        routine.active().onTrue(swerve.applyRequest(() -> drive.withVelocityX(0).
                                                            withVelocityY(0).
                                                            withRotationalRate(0)));
        return routine;
    }

}