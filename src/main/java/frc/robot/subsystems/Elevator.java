package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;
    private DigitalInput elevatorLimitSwitch;

    public elevatorState currentState;
    public enum elevatorState {
        LEVELONEHEIGHT,
        LEVELTWOHEIGHT,
        LEVELTHREEHEIGHT,
        LEVELFOURHEIGHT,
        FEEDER,
        IDLE,
        TEST
    }

    public Elevator() {
        elevatorMotor1 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ONE_ID);
        elevatorMotor2 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_TWO_ID);
        elevatorLimitSwitch = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_ID);

        currentState = elevatorState.IDLE;

        final TalonFXConfiguration elevator1PosConfigs = new TalonFXConfiguration();
        elevator1PosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        elevator1PosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        elevator1PosConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var pivotSlot0Configs = elevator1PosConfigs.Slot0;

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

        //final MotionMagicVoltage m_requestRight = new MotionMagicVoltage(0);
        //m_requestRight.EnableFOC = true;

        DutyCycleOut eDutyCycleOut = new DutyCycleOut(0.0);

        //elevatorMotor1.setControl(eDutyCycleOut.withOutput(speed));
        elevatorMotor2.setControl(eDutyCycleOut.withOutput(elevatorMotor1.get()));
        if(pos == 0 && getElevatorPosition1() <= 0.1 && getLimitSitch() == false) {
            elevatorMotor1.set(-0.2);
        } else {
            elevatorMotor1.setControl(m_requestLeft.withPosition(pos));
        }

        SmartDashboard.putNumber("Elevator2 Error", elevatorMotor2.getClosedLoopError().getValue());
        SmartDashboard.putNumber("Elevator1 Error", elevatorMotor1.getClosedLoopError().getValue());

    }
    public void runElevator(double speed) {
        DutyCycleOut eDutyCycleOut = new DutyCycleOut(0.0);

        //elevatorMotor1.setControl(eDutyCycleOut.withOutput(speed));
        elevatorMotor2.setControl(eDutyCycleOut.withOutput(speed));

    }
    public boolean getLimitSitch() {
        return !elevatorLimitSwitch.get();
    }
    public void setState(elevatorState state) {
        currentState = state;
    }

    public elevatorState getState() {
        return currentState;
    }
    public double getElevatorPosition1() {
        return elevatorMotor1.getRotorPosition().getValueAsDouble();
    }
    public double getElevatorPosition2() {
        return elevatorMotor2.getRotorPosition().getValueAsDouble();
    }
    public void setElevatorDown() {
        elevatorMotor1.setPosition(0);
        elevatorMotor2.setPosition(0);

    }
    public void logging() {
        SmartDashboard.putNumber("Elevator Position1", getElevatorPosition1());
        SmartDashboard.putNumber("Elevator Position2", getElevatorPosition2());

        SmartDashboard.putNumber("Elevator Speed", elevatorMotor1.get());
        SmartDashboard.putBoolean("Elevator Limit Switch", getLimitSitch());
    }
 
    public void periodic() {
        logging();
        if(getLimitSitch()) {
            setElevatorDown();
        }
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
            case TEST:
                runElevator(0.25);
                break;
        }
    }
}
