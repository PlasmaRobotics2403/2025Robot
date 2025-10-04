// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ClimbConstants;
import frc.robot.StateManager.armState;
import frc.robot.StateManager.robotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.robotSideState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private XboxController driver;
  private XboxController coDriver;

  Vision vision = new Vision();
  Climb climb = new Climb();
  Intake intake = new Intake();
  Elevator elevator = new Elevator();
  LEDs leds = new LEDs();
  Arm arm = new Arm();
  StateManager stateManager = new StateManager(intake, elevator, arm, leds, vision/*, climb*/);
  boolean isIntakeing = false;
  double matchTime = 0;
  

  public Robot() {
    driver = new XboxController(0);
    coDriver = new XboxController(1);
    m_robotContainer = new RobotContainer(stateManager, vision);
    // CommandScheduler.getInstance().registerSubsystem(stateManager);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //climb.periodic();
    intake.periodic();
    elevator.periodic();
    arm.periodic();
    leds.periodic();
    vision.update();
    if(DriverStation.isEnabled()) {
      matchTime = DriverStation.getMatchTime();
    }
    SmartDashboard.putNumber("Match Time", matchTime);
    SmartDashboard.putNumber("Climb Pos",   climb.climbMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climb Speed", climb.climbMotor.get());
    SmartDashboard.putBoolean("IsAutoAligning", m_robotContainer.isAutoAligning());
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

    m_robotContainer.configureBindings();

    if(driver.getPOV() == 0) {
      if(climb.climbMotor.getRotorPosition().getValueAsDouble() <= Constants.ClimbConstants.CLIMB_DOWN_POS) {
        climb.runClimb(ClimbConstants.CLIMB_SPEED);
      } else {
        climb.runClimb(0);
      }
      //stateManager.setState(robotState.CLIMBUP);
    } 
    else if(driver.getPOV() == 180) {
      climb.rotClimbPos(ClimbConstants.CLIMB_UP_POS, 1);
      //stateManager.setState(robotState.CLIMBDOWN);
    }
    else if(coDriver.getAButton()) {
      stateManager.setState(robotState.ALGEEHIGH);
      stateManager.setArmState(armState.RUNNINGIN);
    }
    else if(coDriver.getBButton()) {
      stateManager.setState(robotState.ALGEELOW);
      stateManager.setArmState(armState.RUNNINGIN);
    }
    else if(coDriver.getPOV() == 0) {
      climb.runClimb(0.8);
      //climb.setState(climbState.CLIMBDOWNPERCENT);
    }
    else if (coDriver.getPOV() == 180) {
      climb.runClimb(-0.8);
      //climb.setState(climbState.CLIMBUPPERCENT);
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
    else if(driver.getLeftBumperButton() == true) {
      stateManager.setState(robotState.ALGEEIN);
    }
    else if(driver.getBackButton() == true) {
      stateManager.setState(robotState.ALGEEOUT);
    }
    else {
      climb.runClimb(0);
      stateManager.setState(robotState.IDLE);
      stateManager.setArmState(armState.IDLE);
      isIntakeing = false;
    }
    if(driver.getPOV() == 270) {
      vision.setRobotSide(robotSideState.LEFT);
      m_robotContainer.setAutoAligning(true);
    } else if(driver.getPOV() == 90) {
      vision.setRobotSide(robotSideState.RIGHT);
      m_robotContainer.setAutoAligning(true);
    } else {
      vision.setRobotSide(robotSideState.IDLE);
      m_robotContainer.setAutoAligning(false);
    }
    if(driver.getLeftTriggerAxis() >= 0.3) {
      m_robotContainer.setCreeping(0.3);
    } else {
      m_robotContainer.setCreeping(1);
    }
    if(driver.getRightBumperButton() == true) {
        stateManager.setArmState(armState.RUNNINGOUT);
      } else if(isIntakeing == false){
        stateManager.setArmState(armState.IDLE);
    }

    
    
    if(driver.getStartButton() == true) {
      vision.pigeon2.reset();
   }
   if(driver.getStartButton()) {
    m_robotContainer.drivetrain.seedFieldCentric();
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
