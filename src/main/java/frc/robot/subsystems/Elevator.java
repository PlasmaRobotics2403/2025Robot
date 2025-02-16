package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;

    public elevatorState currentState;
    public enum elevatorState {
        LEVELONEHEIGHT,
        LEVELTWOHEIGHT,
        LEVELTHREEHEIGHT,
        LEVELFOURHEIGHT,
        FEEDER,
        IDLE
    }

    public Elevator() {
        elevatorMotor1 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ONE_ID);
        elevatorMotor2 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_TWO_ID);


        currentState = elevatorState.LEVELONEHEIGHT;
        final TalonFXConfiguration elevator1PosConfigs = new TalonFXConfiguration();
        elevator1PosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        elevator1PosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        elevator1PosConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var pivotSlot0Configs = elevator1PosConfigs.Slot0;

        elevator1PosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        elevator1PosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotSlot0Configs.kS = ElevatorConstants.elevatorPosKS;
        pivotSlot0Configs.kV = ElevatorConstants.elevatorPosKV;
        pivotSlot0Configs.kP = ElevatorConstants.elevatorPosKP;
        pivotSlot0Configs.kD = ElevatorConstants.elevatorPosKD;

        
        var pivotMotionMagicConfigs = elevator1PosConfigs.MotionMagic;
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.elevatorPosVel;    //rps
        pivotMotionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.elevatorPosAccel;    //rps/s
        pivotMotionMagicConfigs.MotionMagicJerk = ElevatorConstants.elevatorPosJerk;          //rps/s/s


        final TalonFXConfiguration elevator2PosConfigs = new TalonFXConfiguration();
        elevator2PosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        elevator2PosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        elevator2PosConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var pivotSlot2Configs = elevator2PosConfigs.Slot2;
        elevator2PosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        elevator2PosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotSlot2Configs.kS = ElevatorConstants.elevatorPosKS;
        pivotSlot2Configs.kV = ElevatorConstants.elevatorPosKV;
        pivotSlot2Configs.kP = ElevatorConstants.elevatorPosKP;
        pivotSlot2Configs.kD = ElevatorConstants.elevatorPosKD;

        
        var pivot2MotionMagicConfigs = elevator1PosConfigs.MotionMagic;
        pivot2MotionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.elevatorPosVel;    //rps
        pivot2MotionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.elevatorPosAccel;    //rps/s
        pivot2MotionMagicConfigs.MotionMagicJerk = ElevatorConstants.elevatorPosJerk;          //rps/s/s

        elevatorMotor1.getConfigurator().apply(elevator1PosConfigs);

        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor1.setPosition(0);

        elevatorMotor2.getConfigurator().apply(elevator2PosConfigs);

        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setPosition(0);
    }

    public void magicElevator(double pos) {
        final MotionMagicVoltage m_requestLeft = new MotionMagicVoltage(0);
        m_requestLeft.EnableFOC = true;

        final MotionMagicVoltage m_requestRight = new MotionMagicVoltage(0);
        m_requestRight.EnableFOC = true;

        elevatorMotor1.setControl(m_requestLeft.withPosition(pos));
        elevatorMotor2.setControl(m_requestRight.withPosition(pos));
    }
    public void runElevator() {

    }
    public void setState(elevatorState state) {
        currentState = state;
    }

    public elevatorState getState() {
        return currentState;
    }
    public double getElevatorPosition() {
        return elevatorMotor1.getRotorPosition().getValueAsDouble();
    }
    public void logging() {
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Speed", elevatorMotor1.get());
    }
 
    public void periodic() {
        logging();

        switch (currentState) {
            case IDLE:
                magicElevator(0);
                break;
            case LEVELONEHEIGHT:
                magicElevator(ElevatorConstants.LEVEL1_HEIGHT);
                break;
            case LEVELTWOHEIGHT:
                magicElevator(ElevatorConstants.LEVEL2_HEIGHT);
                break;
            case LEVELTHREEHEIGHT:
                magicElevator(ElevatorConstants.LEVEL3_HEIGHT);
                break;
            case LEVELFOURHEIGHT:
                magicElevator(ElevatorConstants.LEVEL4_HEIGHT);
                break;
            case FEEDER:
                magicElevator(ElevatorConstants.FEEDER_HEIGHT);
                break;
        }
    }
}
