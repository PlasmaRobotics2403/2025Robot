package frc.robot;

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
                elevator.setState(elevatorState.IDLE);
                arm.setIntakeState(armOuttakeState.IDLE);
                arm.setRotState(armRotState.IDLE);

                intakeTimer.reset();
                intakeTimer.stop();
                
                if(elevator.getLimitSitch() && elevator.getElevatorPosition1() <= 0.01) {
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
                    //arm.setRotState(armRotState.MIDPOS);
                } 
                break;
            case LEVELTHREESCORE:
                elevator.setState(elevatorState.LEVELTHREEHEIGHT);
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    arm.setRotState(armRotState.MIDPOS);
                }
                break;
            case LEVELFOURSCORE:
                elevator.setState(elevatorState.LEVELFOURHEIGHT);
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    arm.setRotState(armRotState.HIGHPOS);
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
                    arm.setIntakeState(armOuttakeState.INTAKE);
                }
                else {
                    intake.setState(intakeState.IDLE);
                    arm.setIntakeState(armOuttakeState.IDLE);
                }
                if(!intakeTimer.isRunning() && intake.getIndexSensor()) {
                    intakeTimer.start();
                }
                break;
            case EJECT:
                intake.setState(intakeState.OUTTAKE);
                break;
            case ARMOUTTAKE:
                arm.setIntakeState(armOuttakeState.OUTTAKE);
                break;
            case TESTELEVATOR:
                elevator.setState(elevatorState.TEST);
                break;
        }
    }
}