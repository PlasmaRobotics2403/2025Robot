package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    
    private TalonFX elevatorMotor;

    public elevatorState currentState;
    public enum elevatorState {
        STOWED,
        FIRSTHEIGHT,
        MIDDLEHEIGHT,
        TOPHEIGHT
    }

    public Elevator() {
        elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID);

        currentState = elevatorState.STOWED;
        final TalonFXConfiguration elevatorPosConfigs = new TalonFXConfiguration();
        elevatorPosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        elevatorPosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        var pivotSlot0Configs = elevatorPosConfigs.Slot0;

        elevatorPosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        elevatorPosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotSlot0Configs.kS = ElevatorConstants.elevatorPosKS;
        pivotSlot0Configs.kV = ElevatorConstants.elevatorPosKV;
        pivotSlot0Configs.kP = ElevatorConstants.elevatorPosKP;
        pivotSlot0Configs.kD = ElevatorConstants.elevatorPosKD;

        
        var pivotMotionMagicConfigs = elevatorPosConfigs.MotionMagic;
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.elevatorPosVel;    //rps
        pivotMotionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.elevatorPosAccel;    //rps/s
        pivotMotionMagicConfigs.MotionMagicJerk = ElevatorConstants.elevatorPosJerk;          //rps/s/s

        elevatorMotor.getConfigurator().apply(elevatorPosConfigs);

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setPosition(0);
    }

    public void magicElevator(double pos) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_request.EnableFOC = true;

        elevatorMotor.setControl(m_request.withPosition(pos));
    }

    public void setState(elevatorState state) {
        currentState = state;
    }

    public elevatorState getState() {
        return currentState;
    }

    public void logging() {

    }
 
    public void periodic() {
        logging();

        switch (currentState) {
            case STOWED:
                magicElevator(ElevatorConstants.STOWED_HEIGHT);
                break;
            case FIRSTHEIGHT:
                magicElevator(ElevatorConstants.LOWEST_HEIGHT);
                break;
            case MIDDLEHEIGHT:
                magicElevator(ElevatorConstants.MIDDLE_HEIGHT);
                break;
            case TOPHEIGHT:
                magicElevator(ElevatorConstants.TOP_HEIGHT);
                break;
        }
    }
}
