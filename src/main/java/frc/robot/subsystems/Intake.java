package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class Intake {
    
    public TalonFX intakeMotor;
    public TalonFX rotMotor;
    public TalonSRX indexMotor;
    public DigitalInput indexSensor;

    public boolean isElevatorUp = false;
    public boolean holdingAlgae = false;

    public intakeState currentState;
    
    public enum intakeState {
        IDLE,
        INTAKE,
        OUTTAKE,
        ROTUP,
        ROTDOWN,
        ALGEEIN,
        ALGEEOUT
    }

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, "swerve");
        rotMotor = new TalonFX(IntakeConstants.ROT_MOTOR_ID, "swerve");
        indexMotor = new TalonSRX(IntakeConstants.INDEX_MOTOR_ID);
        indexSensor = new DigitalInput(0);

        currentState = intakeState.IDLE;
        indexMotor.setNeutralMode(NeutralMode.Coast);
        final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rotMotor.getConfigurator().apply(intakeConfigs);

        // pivot motion magic configuration
        final TalonFXConfiguration intakeRotConfigs = new TalonFXConfiguration();
        var pivotSlot0Configs = intakeRotConfigs.Slot0;
        intakeRotConfigs.CurrentLimits.StatorCurrentLimit = 40;
        intakeRotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        

        pivotSlot0Configs.kS = IntakeConstants.intakePivotKS;
        pivotSlot0Configs.kV = IntakeConstants.intakePivotKV;
        pivotSlot0Configs.kP = IntakeConstants.intakePivotKP;
        pivotSlot0Configs.kD = IntakeConstants.intakePivotKD;

        
        var pivotMotionMagicConfigs = intakeRotConfigs.MotionMagic;
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.intakePivotVel;    //rps
        pivotMotionMagicConfigs.MotionMagicAcceleration = IntakeConstants.intakePivotAccel;    //rps/s
        pivotMotionMagicConfigs.MotionMagicJerk = IntakeConstants.intakePivotJerk;          //rps/s/s
        
        
        rotMotor.getConfigurator().apply(intakeRotConfigs);

        rotMotor.setNeutralMode(NeutralModeValue.Brake);
        rotMotor.setPosition(0);

        //Current Limits
        TalonFXConfiguration currentConfigs = new TalonFXConfiguration();

        currentConfigs.CurrentLimits.StatorCurrentLimit = 40;
        currentConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    }
    public void runIntake(double speed) {
        DutyCycleOut intakeRequest = new DutyCycleOut(0.0);
        intakeMotor.setControl(intakeRequest.withOutput(speed));
    }
    public void runIndex(double speed) {
        indexMotor.set(ControlMode.PercentOutput, speed);
    }
    public void rotIntake(double pos) {
        DutyCycleOut intakeRequest = new DutyCycleOut(0.0);
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        if(pos == 0 && getRotPosition() >= -0.4 && getRotPosition() <= 0.4) {
            rotMotor.setControl(intakeRequest.withOutput(0));
        } else {
            rotMotor.setControl(m_request.withPosition(pos));
        }
    }

    public void rotIntakePercent(double speed) {
        rotMotor.set(speed);
    }
    public double getRotPosition() {
        return rotMotor.getRotorPosition().getValueAsDouble();
    }
    public boolean getIndexSensor() {
        return !indexSensor.get();
    }
    public void setState(intakeState state) {
        currentState = state;
    }
    
    public intakeState getState() {
        return currentState;
    }
    public void setAlgaeUp(boolean isUp) {
        holdingAlgae = isUp;
    }
    public void log() {
        SmartDashboard.putNumber("Intake Rot Pos", getRotPosition());
        SmartDashboard.putBoolean("IntakeSensor", getIndexSensor());
    }

    public void periodic() {
        log();
        switch(currentState){
            case IDLE:
                runIntake(0);
                rotIntakePercent(0);
                runIndex(0);
                if(holdingAlgae) {
                    rotIntake(IntakeConstants.INTAKE_ALGEE_POS);
                } else {
                    rotIntake(IntakeConstants.INTAKE_UP_POS);
                }
                break;
            case INTAKE:
                runIndex(IntakeConstants.INDEX_SPEED);
                runIntake(IntakeConstants.INTAKE_SPEED);
                rotIntake(IntakeConstants.INTAKE_DOWN_POS);
                break;
            case OUTTAKE:
                runIntake(-IntakeConstants.INTAKE_SPEED);
                runIndex(-IntakeConstants.INDEX_SPEED);
                rotIntake(IntakeConstants.INTAKE_DOWN_POS);
                break;
            case ROTUP:
                rotIntakePercent(0.3);
                break;
            case ROTDOWN:
                rotIntakePercent(-0.3);
                break;
            case ALGEEIN:
                runIntake(-IntakeConstants.ALGAE_SPEED);
                rotIntake(IntakeConstants.INTAKE_ALGEE_POS);
                break;
            case ALGEEOUT:
                rotIntake(IntakeConstants.INTAKE_UP_POS);
                if(getRotPosition() <= 3) {
                    runIntake(IntakeConstants.ALGAE_SPEED);
                }
                break;

        }
    }
}
