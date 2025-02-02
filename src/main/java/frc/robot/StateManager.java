package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb.climbState;
import frc.robot.subsystems.Intake.intakeState;

public class StateManager extends SubsystemBase {

    // Components of the robot
    public Climb climb;
    public Intake intake;
    public robotState currentState;
    public enum robotState {
        IDLE,
        CLIMBUP,
        CLIMBDOWN,
        TESTINTAKEUP,
        TESTINTAKEDOWN,
        INTAKE,
        OUTTAKE
    }

    public StateManager(Climb climb, Intake intake) {
        currentState = robotState.IDLE;
        this.climb = climb;
        this.intake = intake;
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
            default:
                setState(robotState.IDLE);
        
        }
    }
}
