package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager.levelTwoStates;
import frc.robot.StateManager.robotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.armOuttakeState;
import frc.robot.subsystems.Arm.armRotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.intakeState;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Vision;

public class StateManager extends SubsystemBase {

    // Components of the robot
    //public Climb climb;
    public Intake intake;
    public Elevator elevator;
    public Arm arm;
    public LEDs leds;
    public Vision vision;
    public Timer intakeTimer;
    public Timer elevatorTimer;
    public robotState currentState;
    public armState currentArmState;
    public boolean armUp = false;
    public boolean armLow = false;
    public boolean hasCoral = false;
    public levelTwoStates currentLevelTwoState;
    public enum levelTwoStates {
        IDLE,
        GOINGUP,
        PLACING,
        RETURNING
    }
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
        TESTELEVATOR,
        ALGEEHIGH,
        ALGEELOW,
        ALGEEIN,
        ALGEEOUT

    }

    public StateManager(Intake intake, Elevator elevator, Arm arm, LEDs leds, Vision vision) {
        intakeTimer = new Timer();
        elevatorTimer = new Timer();
        currentState = robotState.IDLE;
        currentArmState = armState.IDLE;
        currentLevelTwoState = levelTwoStates.IDLE;
        this.intake = intake;
        this.elevator = elevator;
        this.arm = arm;
        this.leds = leds;
        this.vision = vision;
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
        return run(
            () -> {
                setState(state);
            });

    }
    public Command setStateL4Command() {
        return run(
            () -> {
                setState(robotState.LEVELFOURSCORE);
            });

    }
    public Command setStateL3Command() {
        return run(
            () -> {
                setState(robotState.LEVELTHREESCORE);
            });

    }
    public Command setArmStateCommand(armState state) {
        return run(
            () -> {
                setArmState(state);
            });
    }
    public void logging() {
        SmartDashboard.putString("Robot State", currentState.toString());
        SmartDashboard.putBoolean("Intake Timer Started", intakeTimer.isRunning());
    }
    public void periodic() {
        logging();
        if(vision.isAutoAligned()) {
            leds.setState(LEDState.ALLIGNED);
        } else if (vision.getApriltagNumber() != 0) {
            leds.setState(LEDState.SEETARGET);
        } else {
            leds.setState(LEDState.NOPEICE);
        }
        switch(currentState) {
            case IDLE:
                armLow = false;
                //climb.setState(climbState.IDLE);
                intake.setState(intakeState.IDLE);
                arm.setRotState(armRotState.IDLE);
                
                if(arm.getRot() <= 0.3 && arm.getRot() >= 0.23) {
                    if(!elevatorTimer.isRunning() && armUp == true) {
                        elevatorTimer.start();
                    }
                    if(elevatorTimer.get() >= 0) {
                        elevator.setState(elevatorState.IDLE);
                        armUp = false;
                        elevatorTimer.stop();
                        elevatorTimer.reset();
                    }
                } else {
                    elevator.setState(elevatorState.IDLEHIGH);
                }

                intakeTimer.reset();
                intakeTimer.stop();
                
                if(elevator.getLimitSitch() && elevator.getElevatorPosition1() <= 0.1) {
                    elevator.setElevatorDown();
                }
                break;
            case LEVELONESCORE:
                armLow = true;
                elevator.setState(elevatorState.LEVELONEHEIGHT);
                arm.setRotState(armRotState.LOWPOS);
                armUp = true;
                
                break;
            case LEVELTWOSCORE:
                armLow = false;
                elevator.setState(elevatorState.LEVELTWOHEIGHT);
                arm.setRotState(armRotState.MIDPOS);
                /*
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    armUp = true;
                    if(arm.getRot() >= Constants.ArmConstants.armMidPos - 0.01 && arm.getRot() <= Constants.ArmConstants.armMidPos + 0.01) {
                        arm.setLimitMotion(false); 
                    } else {
                        arm.setLimitMotion(true);
                    }
                    arm.setRotState(armRotState.MIDPOS);
                } else {
                    arm.setLimitMotion(false);
                }*/ 
                break;
            case LEVELTHREESCORE:
                armLow = false;
                elevator.setState(elevatorState.LEVELTHREEHEIGHT);
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    if(arm.getRot() >= Constants.ArmConstants.armMidPos - 0.01 && arm.getRot() <= Constants.ArmConstants.armMidPos + 0.01) {
                        arm.setLimitMotion(false);
                    } else {
                        arm.setLimitMotion(true);
                    }
                    armUp = true;
                    arm.setRotState(armRotState.LEVELTHREE);
                } else {
                    arm.setLimitMotion(false);
                }
                break;
            case LEVELFOURSCORE:
                armUp = true;
                armLow = false;
                elevator.setState(elevatorState.LEVELFOURHEIGHT);
                if(elevator.getElevatorPosition1() >= Constants.ElevatorConstants.ARM_THRESHHOLD) {
                    if(arm.getRot() >= Constants.ArmConstants.armHighPos - 0.01 && arm.getRot() <= Constants.ArmConstants.armHighPos + 0.01) {
                        arm.setLimitMotion(false);
                    } else {
                        arm.setLimitMotion(true);
                    }
                    armUp = true;
                    arm.setRotState(armRotState.HIGHPOS);
                } else {
                    arm.setLimitMotion(false);
                }
                break;
            case CLIMBUP:
                //climb.setState(climbState.CLIMBUP);
                break;
            case CLIMBDOWN:
                //climb.setState(climbState.CLIMBDOWN);
                break;
            case TESTINTAKEUP:
                intake.setState(intakeState.ROTUP);
                break;
            case TESTINTAKEDOWN:
                intake.setState(intakeState.ROTDOWN);
                break;
            case INTAKE:
                if(arm.getRot() <= 0.3 && arm.getRot() >= 0.23) {
                    if(!elevatorTimer.isRunning() && armUp == true) {
                        elevatorTimer.start();
                    }
                    if(elevatorTimer.get() >= 0) {
                        elevator.setState(elevatorState.IDLE);
                        armUp = false;
                        elevatorTimer.stop();
                        elevatorTimer.reset();
                    }
                } else {
                    elevator.setState(elevatorState.IDLEHIGH);
                }

                if(intakeTimer.get() <= Constants.IntakeConstants.INTAKE_WAIT_TIME) {
                    intake.setState(intakeState.INTAKE);
                }
                else {
                    intake.setState(intakeState.IDLE);
                }
                if(!intakeTimer.isRunning() && intake.getIndexSensor()) {
                    hasCoral = true;
                }
                if(!intakeTimer.isRunning() && !intake.getIndexSensor() && hasCoral == true) {
                    hasCoral = false;
                    intakeTimer.start();
                }
                break;
            case EJECT:
                intake.setState(intakeState.OUTTAKE);
                break;
            case TESTELEVATOR:
                elevator.setState(elevatorState.TEST);
                break;
            case ALGEEHIGH:
                //armLow = true;
                armUp = true;
                elevator.setState(elevatorState.ALLGEEHIGH);
                arm.setRotState(armRotState.ALGEEPOS);
                arm.setIntakeState(armOuttakeState.ALGAE);
                break;
            case ALGEELOW:
                //armLow = true;
                armUp = true;
                elevator.setState(elevatorState.ALGEELOW);
                arm.setRotState(armRotState.ALGEEPOS);
                break;
            case ALGEEIN:
                intake.setAlgaeUp(true);
                intake.setState(intakeState.ALGEEIN);
                break;
            case ALGEEOUT:
                intake.setAlgaeUp(false);
                intake.setState(intakeState.ALGEEOUT);
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
                if(armLow) {
                    arm.setIntakeState(armOuttakeState.SCORELOW);
                } else {
                    arm.setIntakeState(armOuttakeState.OUTTAKE);
                }
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