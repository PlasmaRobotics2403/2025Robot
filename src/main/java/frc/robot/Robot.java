// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.controllers.PlasmaJoystick;
import frc.robot.StateManager.armState;
import frc.robot.StateManager.robotState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb.climbState;
import frc.robot.subsystems.Intake.intakeState;
import frc.robot.subsystems.Vision.robotSideState;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.armOuttakeState;
import frc.robot.subsystems.Arm.armRotState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private XboxController driver;
  
  Vision vision = new Vision();
  Climb climb = new Climb();
  Intake intake = new Intake();
  Elevator elevator = new Elevator();
  Arm arm = new Arm();
  StateManager stateManager = new StateManager(climb, intake, elevator, arm);
  boolean isIntakeing = false;
  

  public Robot() {
    driver = new XboxController(0);
    m_robotContainer = new RobotContainer(stateManager);
  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    climb.periodic();
    intake.periodic();
    elevator.periodic();
    arm.periodic();
    arm.logging();
    stateManager.periodic();
    vision.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    if(driver.getPOV() == 0) {
      stateManager.setState(robotState.CLIMBUP);
      DriverStation.reportWarning("Climb!!!!!!!!!!!", true);
    } 
    else if(driver.getPOV() == 90) {
      stateManager.setState(robotState.LEVELONESCORE);
    }
    else if(driver.getPOV() == 180) {
      stateManager.setState(robotState.CLIMBDOWN);
    }
    else if(driver.getRightTriggerAxis() >= 0.3) {
      stateManager.setState(robotState.INTAKE);
      stateManager.setArmState(armState.INTAKE);
      isIntakeing = true;
    }
    else if(driver.getAButton() == true) {
      stateManager.setState(robotState.LEVELONESCORE);
    }
    else if(driver.getBButton() == true) {
      stateManager.setState(robotState.LEVELTWOSCORE);
    }
    else if(driver.getXButton() == true) {
      stateManager.setState(robotState.LEVELTHREESCORE);
    }
    else if(driver.getYButton() == true) {
      stateManager.setState(robotState.LEVELFOURSCORE);
    }

    else if(driver.getLeftTriggerAxis() >= 0.3) {
      m_robotContainer.setCreeping(0.3);
    }
    else {
      stateManager.setState(robotState.IDLE);
      stateManager.setArmState(armState.IDLE);
      isIntakeing = false;
      m_robotContainer.setCreeping(1);

    }
    if(driver.getRightBumperButton() == true) {
      stateManager.setArmState(armState.RUNNINGOUT);
    } else if(isIntakeing == false){
      stateManager.setArmState(armState.IDLE);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    
  }

}
