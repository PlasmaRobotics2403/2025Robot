// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Swerve drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    private StateManager stateManager;
    private Vision vision;
    public double isCreep = 1;
    public double driveXOutput = 0;
    public double driveYOutput = 0;
    public double driveTurnOutput = 0;
    public boolean isAutoAligning = false;
    public boolean isAngleAligning = false;
    public Trigger startTrigger;

    public RobotContainer(StateManager stateManager, Vision vision) {
        this.stateManager = stateManager;
        this.vision = vision;
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, stateManager, drivetrain, vision);

        startTrigger = joystick.start();
        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("AutoAlign", autoRoutines::autoAlignRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveXOutput) // Drive forward with negative Y (forward)
                    .withVelocityY(driveYOutput) // Drive left with negative X (left)
                    .withRotationalRate(driveTurnOutput) // Drive counterclockwise with negative X (left)
            )
        );
    }

    public void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if(isAutoAligning()) {
            driveXOutput = -vision.moveRobotPoseX() * MaxSpeed;
            driveYOutput = -vision.moveRobotPoseY() * MaxSpeed;
            driveTurnOutput = vision.moveRobotPoseSpin() * MaxAngularRate;
        }
        else{
            driveXOutput = -joystick.getLeftY() * MaxSpeed * isCreep;
            driveYOutput = -joystick.getLeftX() * MaxSpeed * isCreep;
            driveTurnOutput = -joystick.getRightX() * MaxAngularRate * isCreep;
        }
        if(startTrigger.getAsBoolean()) {
            drivetrain.seedFieldCentric();
        }
        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }

    public void setCreeping(double speed) {
        isCreep = speed;
    }
    public boolean isAutoAligning() {

        return isAutoAligning;
    }
    public void setAutoAligning(boolean value) {
        isAutoAligning = value;
    }
    public boolean isAngleAligning() {
        
        return isAngleAligning;
    }
    public void setAngleAligning(boolean value) {
        isAngleAligning = value;
    }
}