
/**
 * 
 */
package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.*;
import frc.lib.autoUtil.Action;
import frc.robot.Constants;
import frc.robot.StateManager;
import frc.robot.StateManager.robotState;
/**
 *
 */
public class AutoRobotState implements Action {

    StateManager manager;
    robotState state;

    public AutoRobotState(StateManager manager, robotState state) {
        this.manager = manager;
        this.state = state;
    }
		
	@Override
	public boolean isFinished() {
        return state == manager.getState();
	}

	@Override
	public void start() {
        manager.setState(state);
	}

	@Override
	public void update() {
	}

	@Override
	public void end() {
	}

}