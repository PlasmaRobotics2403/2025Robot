package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.StateManager.robotState;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final StateManager stateManager;

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
        );
        return routine;
    }
}