package frc.robot;

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

    public robotState currentState;
    public enum robotState {
        IDLE,
        TOPHEIGHT,
        MIDDLEHEIGHT,
        LOWHEIGHT,
        CLIMBUP,
        CLIMBDOWN,
        TESTINTAKEUP,
        TESTINTAKEDOWN,
        INTAKE,
        OUTTAKE,
        ARMFEEDPOS,
        ARMLOWPOS,
        ARMMIDPOS,
        ARMHIGHPOS
    }

    public StateManager(Climb climb, Intake intake, Elevator elevator, Arm arm) {
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
    public void periodic() {
        switch(currentState) {
            case IDLE:
                climb.setState(climbState.IDLE);
                intake.setState(intakeState.IDLE);
                elevator.setState(elevatorState.STOWED);

                arm.setIntakeState(armOuttakeState.IDLE);
                arm.setRotState(armRotState.IDLE);
                break;
            case TOPHEIGHT:
                elevator.setState(elevatorState.TOPHEIGHT);
                break;
            case MIDDLEHEIGHT:
                elevator.setState(elevatorState.MIDDLEHEIGHT);
                break;
            case LOWHEIGHT:
                elevator.setState(elevatorState.FIRSTHEIGHT);
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
                intake.setState(intakeState.INTAKE);
                break;
            case OUTTAKE:
                intake.setState(intakeState.OUTTAKE);
                break;
            case ARMMIDPOS:
                arm.setRotState(armRotState.MIDPOS);
                break;
        }
    }
}
