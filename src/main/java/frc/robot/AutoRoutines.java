package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager.armState;
import frc.robot.StateManager.robotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.robotSideState;

public class AutoRoutines extends SubsystemBase{
    private AutoFactory m_factory;
    private StateManager stateManager;
    private RobotContainer robotContainer;
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

    // Auto Routines
    final AutoRoutine twoPieceNearRoutine;
    final AutoRoutine twoPieceFarRoutine;
    final AutoRoutine onePieceFarRoutine;
    final AutoRoutine onePieceNearRoutine;

    // Autos
    private AutoTrajectory twoPieceNear1;
    private AutoTrajectory twoPieceNear2;
    private AutoTrajectory twoPieceNear3;

    private AutoTrajectory twoPieceFar1;
    private AutoTrajectory twoPieceFar2;
    private AutoTrajectory twoPieceFar3;

    private AutoTrajectory onePieceFar1;

    private AutoTrajectory onePieceNear1;

    public AutoRoutines(AutoFactory factory, StateManager stateManager, Swerve swerve, Vision vision, RobotContainer robotContainer) {
        
        m_factory = factory;
        this.stateManager = stateManager;
        this.swerve = swerve;
        this.vision = vision;
        this.robotContainer = robotContainer;

        twoPieceNearRoutine = m_factory.newRoutine("twoPieceNear");
        twoPieceFarRoutine = m_factory.newRoutine("twoPieceFar");
        onePieceFarRoutine = m_factory.newRoutine("onePieceFar");
        onePieceNearRoutine = m_factory.newRoutine("onePieceNear");
        
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            // Two Piece Near Blue
            twoPieceNear1 = twoPieceNearRoutine.trajectory("BlueNearLeftpart1");
            twoPieceNear2 = twoPieceNearRoutine.trajectory("BlueNearLeftpart2");
            twoPieceNear3 = twoPieceNearRoutine.trajectory("BlueNearLeftpart3");

            // Two Piece Far Blue
            twoPieceFar1 = twoPieceFarRoutine.trajectory("RedFarLeftpart1");
            twoPieceFar2 = twoPieceFarRoutine.trajectory("RedFarLeftpart2");
            twoPieceFar3 = twoPieceFarRoutine.trajectory("RedFarLeftpart3");

            // One Piece Far Blue
            onePieceFar1 = onePieceFarRoutine.trajectory("Blue1PieceFar");

            // One Piece Near Blue
            onePieceNear1 = onePieceNearRoutine.trajectory("Blue1PieceNear");

        } else {
             // Two Piece Near Red
            twoPieceNear1 = twoPieceNearRoutine.trajectory("RedNearLeftpart1");
            twoPieceNear2 = twoPieceNearRoutine.trajectory("RedNearLeftpart2");
            twoPieceNear3 = twoPieceNearRoutine.trajectory("RedNearLeftpart3");

            // Two Piece Far Red
            twoPieceFar1 = twoPieceFarRoutine.trajectory("RedFarLeftpart1");
            twoPieceFar2 = twoPieceFarRoutine.trajectory("RedFarLeftpart2");
            twoPieceFar3 = twoPieceFarRoutine.trajectory("RedFarLeftpart3");

            // One Piece Far Red
            onePieceFar1 = onePieceFarRoutine.trajectory("Red1PieceFar");

            // One Piece Near Red
            onePieceNear1 = onePieceNearRoutine.trajectory("Red1PieceNear");
        }

        
    }

    public Command autoAlignCommand(boolean algignLeft) {
        Command alignToTarget = new RunCommand(() -> {
            if(algignLeft == true) {
                vision.setRobotSide(robotSideState.LEFT);
            } else {
                vision.setRobotSide(robotSideState.RIGHT);
            }
            robotContainer.setAutoAligning(true);
            robotContainer.configureBindings();
            // double xOutput = vision.moveRobotPoseX() * MaxSpeed;
            // double yOutput = vision.moveRobotPoseY() * MaxSpeed;
            // double spinOutput = vision.moveRobotPoseSpin() * MaxAngularRate;
            // swerve.setDefaultCommand((swerve.applyRequest(() -> drive.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(spinOutput))));
        });
        return alignToTarget;
    }
    public Command waitCommand(long delayMillis) {
        return run(
            () -> {
                try {
                    Thread.sleep(delayMillis);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            });
    }

    public void loopPath(AutoTrajectory path) {
        stateManager.setState(robotState.INTAKE);
        while (stateManager.hasCoral == false) {
            if(path.active().getAsBoolean() == false){
                path.cmd();
            }
        }
    }
    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("AutoAlignPath");
        routine.active().onTrue(
            new SequentialCommandGroup(
                runOnce(()->swerve.resetPose(new Pose2d(7.6, 4.5, new Rotation2d()))),
                swerve.applyRequest(()->swerve.driveToPos(new Pose2d(6.5, 4.5, new Rotation2d()))).until(()->swerve.isWithinPos(new Pose2d(6.5, 4.5, new Rotation2d()))),
                robotContainer.drive(0, 0, 0).withTimeout(0.1)
            )
        );
        
        return routine;
    }
    
    

public AutoRoutine onePieceMid() {
        final AutoRoutine onePieceMidRoutine = m_factory.newRoutine("Auto Align");

        onePieceMidRoutine.active().onTrue(
            new SequentialCommandGroup(
                runOnce(()->swerve.resetPose(new Pose2d(7.6, 4.5, new Rotation2d()))),
                //path1.cmd(),
                swerve.applyRequest(()->swerve.driveToPos(new Pose2d(6.5, 4.5, new Rotation2d()))).until(()->swerve.isWithinPos(new Pose2d(6.5, 4.5, new Rotation2d()))),
                robotContainer.drive(0, 0, 0).withTimeout(0.1),
                autoAlignCommand(true).withTimeout(2),
                // stateManager.setStateL3Command().alongWith(waitCommand(1000)).withTimeout(4),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),


                robotContainer.drive(0, 0, 0)
            )
        );

        return onePieceMidRoutine;
    }

    public AutoRoutine onePieceNear() {
        boolean alginLeft = false;

        onePieceNearRoutine.active().onTrue(
            new SequentialCommandGroup(
                onePieceNear1.resetOdometry(),
                onePieceNear1.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.1),
                autoAlignCommand(alginLeft).withTimeout(2),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                robotContainer.drive(0, 0, 0).withTimeout(0.01)
            )
        );

        return onePieceNearRoutine;
    }

    public AutoRoutine onePieceFar() {
        boolean alginLeft = true;
        onePieceFarRoutine.active().onTrue(
            new SequentialCommandGroup(
                onePieceFar1.resetOdometry(),
                onePieceFar1.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.1),
                autoAlignCommand(alginLeft).withTimeout(2),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),

                robotContainer.drive(0, 0, 0)
            )
        );

        return onePieceFarRoutine;
    }

    
    public AutoRoutine twoPieceFar() {
        boolean alginLeft = true;

        twoPieceFarRoutine.active().onTrue(
            new SequentialCommandGroup(
                twoPieceFar1.resetOdometry(),
                twoPieceFar1.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                autoAlignCommand(alginLeft).withTimeout(1.5),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(0.5),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                run(() ->stateManager.setArmState(armState.INTAKE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.INTAKE)).withTimeout(1),
                twoPieceFar2.cmd(),
                twoPieceFar3.cmd(),
                //run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                //run(() ->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                autoAlignCommand(alginLeft).withTimeout(1),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(0.5),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                robotContainer.drive(0, 0, 0).withTimeout(0.01)
            )
        );

        return twoPieceFarRoutine;
    }

    public AutoRoutine twoPieceNear() {
        boolean alginLeft = true;

        twoPieceNearRoutine.active().onTrue(
            new SequentialCommandGroup(
                twoPieceNear1.resetOdometry(),
                twoPieceNear1.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                autoAlignCommand(alginLeft).withTimeout(1.5),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(0.5),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                run(() ->stateManager.setArmState(armState.INTAKE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.INTAKE)).withTimeout(1),
                twoPieceNear2.cmd(),
                twoPieceNear3.cmd(),
                //run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                //run(() ->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                autoAlignCommand(alginLeft).withTimeout(1),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(1.5),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(0.5),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                robotContainer.drive(0, 0, 0).withTimeout(0.01)
            )
        );

        return twoPieceNearRoutine;
    }

    public void cacheAutos() {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            // Two Piece Near Blue
            twoPieceNear1 = twoPieceNearRoutine.trajectory("BlueNearLeftpart1");
            twoPieceNear2 = twoPieceNearRoutine.trajectory("BlueNearLeftpart2");
            twoPieceNear3 = twoPieceNearRoutine.trajectory("BlueNearLeftpart3");

            // Two Piece Far Blue
            twoPieceFar1 = twoPieceFarRoutine.trajectory("RedFarLeftpart1");
            twoPieceFar2 = twoPieceFarRoutine.trajectory("RedFarLeftpart2");
            twoPieceFar3 = twoPieceFarRoutine.trajectory("RedFarLeftpart3");

            // One Piece Far Blue
            onePieceFar1 = onePieceFarRoutine.trajectory("Blue1PieceFar");

            // One Piece Near Blue
            onePieceNear1 = onePieceNearRoutine.trajectory("Blue1PieceNear");

        } else {
             // Two Piece Near Red
            twoPieceNear1 = twoPieceNearRoutine.trajectory("RedNearLeftpart1");
            twoPieceNear2 = twoPieceNearRoutine.trajectory("RedNearLeftpart2");
            twoPieceNear3 = twoPieceNearRoutine.trajectory("RedNearLeftpart3");

            // Two Piece Far Red
            twoPieceFar1 = twoPieceFarRoutine.trajectory("RedFarLeftpart1");
            twoPieceFar2 = twoPieceFarRoutine.trajectory("RedFarLeftpart2");
            twoPieceFar3 = twoPieceFarRoutine.trajectory("RedFarLeftpart3");

            // One Piece Far Red
            onePieceFar1 = onePieceFarRoutine.trajectory("Red1PieceFar");

            // One Piece Near Red
            onePieceNear1 = onePieceNearRoutine.trajectory("Red1PieceNear");
        }

    }

}

