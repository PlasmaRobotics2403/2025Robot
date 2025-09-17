package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;

public class Climb {
    public TalonFX climbMotor;
    private MotionMagicVoltage rotClimbRequest;
    DutyCycleOut climbRequest = new DutyCycleOut(0.0);

    private double lastPos = 0;
    public climbState currentState;
    public enum climbState{
        IDLE,
        CLIMBUPPERCENT,
        CLIMBDOWNPERCENT,
        CLIMBUP,
        CLIMBDOWN
    }

    public Climb() {
        climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
        climbMotor.setPosition(0);
        currentState = climbState.IDLE;
        rotClimbRequest = new MotionMagicVoltage(0);

        final TalonFXConfiguration climbPosConfigs = new TalonFXConfiguration();
        climbPosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        climbPosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        climbPosConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbPosConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var pivotSlot0Configs = climbPosConfigs.Slot0;

        pivotSlot0Configs.kS = ClimbConstants.climbPosKSSlot0;
        pivotSlot0Configs.kV = ClimbConstants.climbPosKVSlot0;
        pivotSlot0Configs.kP = ClimbConstants.climbPosKPSlot0;
        pivotSlot0Configs.kD = ClimbConstants.climbPosKDSlot0;

        var pivotSlot1Configs = climbPosConfigs.Slot1;

        pivotSlot1Configs.kS = ClimbConstants.climbPosKSSlot1;
        pivotSlot1Configs.kV = ClimbConstants.climbPosKVSlot1;
        pivotSlot1Configs.kP = ClimbConstants.climbPosKPSlot1;
        pivotSlot1Configs.kD = ClimbConstants.climbPosKDSlot1;

        var pivotMotionMagicConfigs = climbPosConfigs.MotionMagic;
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.climbPosVel;    //rps
        pivotMotionMagicConfigs.MotionMagicAcceleration = ClimbConstants.climbPosAccel;    //rps/s
        pivotMotionMagicConfigs.MotionMagicJerk = ClimbConstants.climbPosJerk;          //rps/s/s
        

        climbMotor.getConfigurator().apply(climbPosConfigs);
    }
    public void rotClimbPos(double pos, int slot) {
        rotClimbRequest.Slot = slot;
        SmartDashboard.putNumber("Arm Commanded Pos", pos);
        // if(climbMotor.getRotorPosition().getValueAsDouble() <= ClimbConstants.CLIMB_UP_POS - 10) {
        //     climbMotor.set(0);
        // } else if(climbMotor.getRotorPosition().getValueAsDouble() >= ClimbConstants.CLIMB_DOWN_POS + 10) {
        //     climbMotor.set(0);
        // }
        //  else {
        //     climbMotor.setControl(rotClimbRequest.withPosition(pos));
        // }
        climbMotor.setControl(rotClimbRequest.withPosition(pos));
    }
    public void setState(climbState state) {
        currentState = state;
    }

    public climbState getState() {
        return currentState;
    }
    public void runClimb(double speed) {
        //DutyCycleOut climbRequest = new DutyCycleOut(0.0);
        climbMotor.setControl(climbRequest.withOutput(speed));
    }
    public void setLastPos(double pos) {
        lastPos = pos;
    }
    private void logging() {
        SmartDashboard.putNumber("Climb Pos", climbMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb Speed", climbMotor.get());
    }
    public void periodic() {
        logging();
        switch (currentState) {
            case IDLE:
                runClimb(0);
                break;
            case CLIMBUP:
                // //rotClimbPos(ClimbConstants.CLIMB_DOWN_POS, 0);
                // if(climbMotor.getRotorPosition().getValueAsDouble() >= ClimbConstants.CLIMB_UP_POS) {
                //     runClimb(ClimbConstants.CLIMB_SPEED);
                // } else {
                //     runClimb(0);
                // }
                break;
            case CLIMBDOWN:
                rotClimbPos(ClimbConstants.CLIMB_UP_POS, 1);
                break;
            case CLIMBUPPERCENT:
                DriverStation.reportWarning("ClimbUp", false);
                runClimb(ClimbConstants.CLIMB_SPEED);
                break;
            case CLIMBDOWNPERCENT:
                DriverStation.reportWarning("ClimbDown", false);
                runClimb(-ClimbConstants.CLIMB_SPEED);
                break;
        }
    }

}
