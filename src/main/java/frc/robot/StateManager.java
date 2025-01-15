package frc.robot;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.climbState;

public class StateManager {

    // Components of the robot
    public Climb climb;

    public robotState currentState;
    public enum robotState {
        IDLE,
        CLIMBUP,
        CLIMBDOWN
    }

    public StateManager(Climb climb) {
        currentState = robotState.IDLE;
        this.climb = climb;
    }

    public void setState(robotState state) {
        currentState = state;
    }
    public robotState getState() {
        return currentState;
    }

    public void periodic() {
        switch(currentState) {
            case IDLE:
                climb.setState(climbState.IDLE);
                break;
            case CLIMBUP:
                climb.setState(climbState.CLIMBUP);
                break;
            case CLIMBDOWN:
                climb.setState(climbState.CLIMBDOWN);
                break;
            default:
                setState(robotState.IDLE);
        
        }
    }
}
