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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.StateManager.armState;
import frc.robot.StateManager.robotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.robotSideState;

public class AutoRoutines extends SubsystemBase{
    private AutoFactory m_factory = null;
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

    public AutoRoutines(AutoFactory factory, StateManager stateManager, Swerve swerve, Vision vision, RobotContainer robotContainer) {
        m_factory = factory;
        this.stateManager = stateManager;
        this.swerve = swerve;
        this.vision = vision;
        this.robotContainer = robotContainer;
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
    public AutoRoutine twoPieceAutoRed() {
        final AutoRoutine routine = m_factory.newRoutine("Auto Align");
        final AutoTrajectory path1 = routine.trajectory("RedMiddleLeft2part1");
        final AutoTrajectory path2 = routine.trajectory("RedMiddleLeft2part2");
        final AutoTrajectory path3 = routine.trajectory("RedMiddleLeft2part3");


        routine.active().onTrue(
            new SequentialCommandGroup(
                path1.resetOdometry(),
                path1.cmd(),
                // swerve.applyRequest(()->swerve.driveToPos(new Pose2d(6.5, 4.5, new Rotation2d()))).until(()->swerve.isWithinPos(new Pose2d(6.5, 4.5, new Rotation2d()))),
                // robotContainer.drive(0, 0, 0).withTimeout(0.1),
                autoAlignCommand(true).withTimeout(2),
                // stateManager.setStateL3Command().alongWith(waitCommand(1000)).withTimeout(4),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(1),

                path2.cmd().alongWith(new SequentialCommandGroup(new WaitCommand(2), run(() -> stateManager.setState(robotState.INTAKE)).withTimeout(1))),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                path3.cmd(),
                autoAlignCommand(true).withTimeout(1.5),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                robotContainer.drive(0, 0, 0)

            )
        );

        return routine;
    }
    public AutoRoutine twoPieceAutoBlue() {
        final AutoRoutine routine = m_factory.newRoutine("Auto Align");
        final AutoTrajectory path1 = routine.trajectory("BlueMiddleLeft2part1");
        final AutoTrajectory path2 = routine.trajectory("BlueMiddleLeft2part2");
        final AutoTrajectory path3 = routine.trajectory("BlueMiddleLeft2part3");


        routine.active().onTrue(
            new SequentialCommandGroup(
                runOnce(()->swerve.resetPose(new Pose2d(7.2, 4.2, new Rotation2d()))),
                swerve.applyRequest(()->swerve.driveToPos(new Pose2d(6.2, 4.2, new Rotation2d()))).until(()->swerve.isWithinPos(new Pose2d(6.5, 4.5, new Rotation2d()))).withTimeout(2),
                robotContainer.drive(0, 0, 0).withTimeout(0.1),
                autoAlignCommand(true).withTimeout(2),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(1),

                path2.cmd().alongWith(new SequentialCommandGroup(new WaitCommand(2), run(() -> stateManager.setState(robotState.INTAKE)).withTimeout(1))),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                path3.cmd(),
                autoAlignCommand(true).withTimeout(1.5),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                robotContainer.drive(0, 0, 0).withTimeout(0.1)

            )
        );

        return routine;
    }

public AutoRoutine onePieceMid() {
        final AutoRoutine routine = m_factory.newRoutine("Auto Align");
        final AutoTrajectory path1 = routine.trajectory("AutoAlignPath");


        routine.active().onTrue(
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

        return routine;
    }

    public AutoRoutine onePieceNear() {
        final AutoRoutine routine = m_factory.newRoutine("onePieceNear");
        AutoTrajectory selectedPath;
        boolean alginLeft;
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            selectedPath = routine.trajectory("Blue1PieceNear");
            alginLeft = true;
        } else {
            selectedPath = routine.trajectory("Red1PieceNear");
            alginLeft = true;
        }

        routine.active().onTrue(
            new SequentialCommandGroup(
                selectedPath.resetOdometry(),
                selectedPath.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.1),
                autoAlignCommand(alginLeft).withTimeout(2),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),


                robotContainer.drive(0, 0, 0).withTimeout(0.01)
            )
        );

        return routine;
    }

    public AutoRoutine onePieceFar() {
        final AutoRoutine routine = m_factory.newRoutine("onePieceFar");
        AutoTrajectory selectedPath;
        boolean alginLeft;
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            selectedPath = routine.trajectory("Blue1PieceFar");
            alginLeft = true;
        } else {
            selectedPath = routine.trajectory("Red1PieceFar");
            alginLeft = true;
        }

        routine.active().onTrue(
            new SequentialCommandGroup(
                selectedPath.resetOdometry(),
                selectedPath.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.1),
                autoAlignCommand(alginLeft).withTimeout(2),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(1),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),


                robotContainer.drive(0, 0, 0)
            )
        );

        return routine;
    }

    
    public AutoRoutine twoPieceFar() {
        final AutoRoutine routine = m_factory.newRoutine("twoPieceFar");
        AutoTrajectory selectedPath1;
        AutoTrajectory selectedPath2;
        AutoTrajectory selectedPath3;
        AutoTrajectory selectedPath4;

        boolean alginLeft;
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            selectedPath1 = routine.trajectory("BlueFarLeftpart1");
            selectedPath2 = routine.trajectory("BlueFarLeftpart2");
            selectedPath3 = routine.trajectory("BlueFarLeftpart3");

            alginLeft = true;
        } else {
            selectedPath1 = routine.trajectory("RedFarLeftpart1");
            selectedPath2 = routine.trajectory("RedFarLeftpart2");
            selectedPath3 = routine.trajectory("RedFarLeftpart3");

            alginLeft = true;
        }

        routine.active().onTrue(
            new SequentialCommandGroup(
                selectedPath1.resetOdometry(),
                selectedPath1.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                autoAlignCommand(alginLeft).withTimeout(1.5),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(0.5),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                run(() ->stateManager.setArmState(armState.INTAKE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.INTAKE)).withTimeout(1),
                selectedPath2.cmd(),
                selectedPath3.cmd(),
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

        return routine;
    }

    public AutoRoutine twoPieceNear() {
        final AutoRoutine routine = m_factory.newRoutine("twoPieceNear");
        AutoTrajectory selectedPath1;
        AutoTrajectory selectedPath2;
        AutoTrajectory selectedPath3;
        AutoTrajectory selectedPath4;

        boolean alginLeft;
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            selectedPath1 = routine.trajectory("BlueNearLeftpart1");
            selectedPath2 = routine.trajectory("BlueNearLeftpart2");
            selectedPath3 = routine.trajectory("BlueNearLeftpart3");

            alginLeft = true;
        } else {
            selectedPath1 = routine.trajectory("RedNearLeftpart1");
            selectedPath2 = routine.trajectory("RedNearLeftpart2");
            selectedPath3 = routine.trajectory("RedNearLeftpart3");

            alginLeft = true;
        }

        routine.active().onTrue(
            new SequentialCommandGroup(
                selectedPath1.resetOdometry(),
                selectedPath1.cmd(),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                autoAlignCommand(alginLeft).withTimeout(1.5),
                robotContainer.drive(0, 0, 0).withTimeout(0.01),
                run(()->stateManager.setState(robotState.LEVELFOURSCORE)).withTimeout(2),
                run(()->stateManager.setArmState(armState.RUNNINGOUT)).withTimeout(0.5),
                run(()->stateManager.setArmState(armState.IDLE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.IDLE)).withTimeout(0.01),
                run(() ->stateManager.setArmState(armState.INTAKE)).withTimeout(0.01),
                run(()->stateManager.setState(robotState.INTAKE)).withTimeout(1),
                selectedPath2.cmd(),
                selectedPath3.cmd(),
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

        return routine;
    }

}

