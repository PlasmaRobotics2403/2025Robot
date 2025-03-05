package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.armOuttakeState;
import frc.robot.subsystems.Arm.armRotState;
import frc.robot.subsystems.Climb.climbState;
import frc.robot.subsystems.Elevator.elevatorState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.intakeState;

public class StateManager extends SubsystemBase {

    // Components of the robot
    public Climb climb;
    public Intake intake;
    public Elevator elevator;
    public Arm arm;
    public Timer intakeTimer;
    public robotState currentState;
    public armState currentArmState;
    public enum armState {
        IDLE,
        RUNNINGIN,
        RUNNINGOUT,
        INTAKE
    }
    public enum robotState {
        IDLE,
        LEVELFOURSCORE,
        LEVELTHREESCORE,
        LEVELTWOSCORE,
        LEVELONESCORE,
        FEEDER,
        CLIMBUP,
        CLIMBDOWN,
        TESTINTAKEUP,
        TESTINTAKEDOWN,
        INTAKE,
        EJECT,
        ARMOUTTAKE,
        TESTELEVATOR

    }

    public StateManager(Climb climb, Intake intake, Elevator elevator, Arm arm) {
        intakeTimer = new Timer();
        currentState = robotState.IDLE;
        currentArmState = armState.IDLE;
        this.climb = climb;
        this.intake = intake;
        this.elevator = elevator;
        this.arm = arm;
    }

    public void setState(robotState state) {
        currentState = state;
    }

    public robotState getState() {
        return currentState;
    }

    public void setArmState(armState state) {
        currentArmState = state;
    }

    public armState getArmState() {
        return currentArmState;
    }

    public Command setStateCommand(robotState state) {
        return runOnce(
            () -> {
                setState(state);
            });

    }
    public void logging() {
        SmartDashboard.putBoolean("Intake Timer Started", intakeTimer.isRunning());
    }
    public void periodic() {
        logging();
        switch(currentState) {
            case IDLE:
                climb.setState(climbState.IDLE);
                intake.setState(intakeState.IDLE);
                
                if(arm.getRot() <= 0.2) {
                    elevator.setState(elevatorState.IDLE);
                }
                arm.setRotState(armRotState.IDLE);

                intakeTimer.reset();
                intakeTimer.stop();
                
                if(elevator.getLimitSitch() && elevator.getElevatorPosition1() <= 0.1) {
                    elevator.setElevatorDown();
                }
                break;
            case LEVELONESCORE:
                elevator.setState(elevatorState.LEVELONEHEIGHT);
                arm.setRotState(armRotState.LOWPOS);
                break;
            case LEVELTWOSCORE:
                elevator.setState(elevatorState.LEVELTWOHEIGHT);
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    if(arm.getRot() >= Constants.ArmConstants.armMidPos - 1 && arm.getRot() <= Constants.ArmConstants.armMidPos + 1) {
                        arm.setLimitMotion(false);
                    } else {
                        arm.setLimitMotion(true);
                    }
                    arm.setRotState(armRotState.MIDPOS);
                } else {
                    arm.setLimitMotion(false);
                }
                break;
            case LEVELTHREESCORE:
                elevator.setState(elevatorState.LEVELTHREEHEIGHT);
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    if(arm.getRot() >= Constants.ArmConstants.armMidPos - 1 && arm.getRot() <= Constants.ArmConstants.armMidPos + 1) {
                        arm.setLimitMotion(false);
                    } else {
                        arm.setLimitMotion(true);
                    }
                    arm.setRotState(armRotState.MIDPOS);
                } else {
                    arm.setLimitMotion(false);
                }
                break;
            case LEVELFOURSCORE:
                elevator.setState(elevatorState.LEVELFOURHEIGHT);
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    if(arm.getRot() >= Constants.ArmConstants.armHighPos - 1 && arm.getRot() <= Constants.ArmConstants.armHighPos + 1) {
                        arm.setLimitMotion(false);
                    } else {
                        arm.setLimitMotion(true);
                    }
                    arm.setRotState(armRotState.HIGHPOS);
                } else {
                    arm.setLimitMotion(false);
                }
                break;
            case CLIMBUP:
                climb.setState(climbState.CLIMBUP);
                break;
            case CLIMBDOWN:
                climb.setState(climbState.CLIMBDOWN);
                break;
            case TESTINTAKEUP:
                intake.setState(intakeState.ROTUP);
                break;
            case TESTINTAKEDOWN:
                intake.setState(intakeState.ROTDOWN);
                break;
            case INTAKE:
                if(intakeTimer.get() <= Constants.IntakeConstants.INTAKE_WAIT_TIME) {
                    intake.setState(intakeState.INTAKE);
                }
                else {
                    intake.setState(intakeState.IDLE);
                }
                if(!intakeTimer.isRunning() && intake.getIndexSensor()) {
                    intakeTimer.start();
                }
                break;
            case EJECT:
                intake.setState(intakeState.OUTTAKE);
                break;
            case TESTELEVATOR:
                elevator.setState(elevatorState.TEST);
                break;

        
        }
        switch (currentArmState) {
            case IDLE:
                arm.setIntakeState(armOuttakeState.IDLE);
                break;
            case RUNNINGIN:
                arm.setIntakeState(armOuttakeState.INTAKE);
                break;
            case RUNNINGOUT:
                arm.setIntakeState(armOuttakeState.OUTTAKE);
                break;
            case INTAKE:
                if(intakeTimer.get() <= Constants.IntakeConstants.INTAKE_WAIT_TIME) {
                    arm.setIntakeState(armOuttakeState.INTAKE);
                } else {
                    arm.setIntakeState(armOuttakeState.IDLE);
                }
                break;
                
        }
    }
}