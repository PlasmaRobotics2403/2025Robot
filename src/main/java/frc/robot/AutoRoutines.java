package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.lib.autoUtil.AutoMode;
import frc.robot.StateManager.robotState;
import frc.robot.auto.actions.AutoRobotState;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private StateManager stateManager;

    public AutoRoutines(AutoFactory factory, StateManager stateManager) {
        m_factory = factory;
        this.stateManager = stateManager;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");
    
        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
                .andThen(stateManager.setStateCommand(robotState.IDLE))
        );
        
        return routine;
    }
    public AutoRoutine testAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Test Auto");
        routine.active().onTrue(stateManager.setStateCommand(null));

        return routine;
    }
}