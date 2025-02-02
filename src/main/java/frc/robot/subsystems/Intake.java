package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class Intake {
    
    public TalonFX intakeMotor;
    public TalonFX rotMotor;

    public intakeState currentState;
    
    public enum intakeState {
        IDLE,
        INTAKE,
        OUTTAKE,
        ROTUP,
        ROTDOWN
    }

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
        rotMotor = new TalonFX(IntakeConstants.ROT_MOTOR_ID);
        currentState = intakeState.IDLE;
        /*/
        var intakeVelocityConfigs = new TalonFXConfiguration();

        var velocitySlot0Configs = intakeVelocityConfigs.Slot0;
        velocitySlot0Configs.kS = IntakeConstants.intakeVelocityKS;
        velocitySlot0Configs.kV = IntakeConstants.intakeVelocityKV;
        velocitySlot0Configs.kA = IntakeConstants.intakeVelocityKA;
        velocitySlot0Configs.kP = IntakeConstants.intakeVelocityKP;
        velocitySlot0Configs.kI = IntakeConstants.intakeVelocityKI;
        velocitySlot0Configs.kD = IntakeConstants.intakeVelocityKD;

        var velocityMotionMagicConfigs = intakeVelocityConfigs.MotionMagic;
        velocityMotionMagicConfigs.MotionMagicAcceleration = IntakeConstants.intakeVelocityAccel;    //rps/s
        velocityMotionMagicConfigs.MotionMagicJerk = IntakeConstants.intakeVelocityJerk;             //rps/s/s

        intakeMotor.getConfigurator().apply(intakeVelocityConfigs);
        */
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

    public void runIntake(double rps) {
        final MotionMagicVelocityVoltage r_request = new MotionMagicVelocityVoltage(0);
        r_request.Acceleration = 0;
        r_request.EnableFOC = true;

        intakeMotor.setControl(r_request.withVelocity(rps));
    }
    public void runIntakeTest(double speed) {
        final DutyCycleOut r_request = new DutyCycleOut(0);
        r_request.EnableFOC = true;
        intakeMotor.setControl(r_request.withOutput(speed));
    }
    public void rotIntake(double pos) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        rotMotor.setControl(m_request.withPosition(pos));
    }

    public void rotIntakePercent(double speed) {
        rotMotor.set(speed);
    }
    public double getRotPosition() {
        return rotMotor.getRotorPosition().getValueAsDouble();
    }

    public void setState(intakeState state) {
        currentState = state;
    }
    
    public intakeState getState() {
        return currentState;
    }

    public void log() {
        SmartDashboard.putNumber("Intake Rot Pos", getRotPosition());
        
    }

    public void periodic() {
        log();

        switch(currentState){
            case IDLE:
                runIntakeTest(0);
                rotIntakePercent(0);
                //rotIntake(IntakeConstants.INTAKE_UP_POS);
                break;
            case INTAKE:
                runIntakeTest(IntakeConstants.INTAKE_SPEED);
                DriverStation.reportWarning("Intaking!!", false);
                //rotIntake(IntakeConstants.INTAKE_DOWN_POS);
                break;
            case OUTTAKE:
                runIntakeTest(-IntakeConstants.INTAKE_SPEED);
                DriverStation.reportWarning("Intaking!!", false);

                //rotIntake(IntakeConstants.INTAKE_DOWN_POS);
                break;
            case ROTUP:
                rotIntakePercent(0.3);
                break;
            case ROTDOWN:
                rotIntakePercent(-0.3);
                break;
        }
    }
}
