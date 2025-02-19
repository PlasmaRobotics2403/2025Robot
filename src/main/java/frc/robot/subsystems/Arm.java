package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class Arm {
    private TalonFX rotMotor;
    private TalonFX endEffectorMotor;
    private CANcoder armEncoder;

    public armOuttakeState currentOuttakeState;
    public enum armOuttakeState {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    public armRotState currentRotState;
    public enum armRotState {
        IDLE,
        FEEDPOS,
        LOWPOS,
        MIDPOS,
        HIGHPOS
    }
    public Arm() {
        rotMotor = new TalonFX(ArmConstants.armRotID);
        endEffectorMotor = new TalonFX(ArmConstants.armOuttakeID);
        armEncoder = new CANcoder(ArmConstants.armCancoderID, "rio");

        currentOuttakeState = armOuttakeState.IDLE;
        currentRotState = armRotState.IDLE;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        armEncoder.getConfigurator().apply(encoderConfig);

        final TalonFXConfiguration armPosConfigs = new TalonFXConfiguration();
        armPosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        armPosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        armPosConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armPosConfigs.Feedback.FeedbackRemoteSensorID = ArmConstants.armCancoderID;
        armPosConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        var pivotSlot0Configs = armPosConfigs.Slot0;

        armPosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        armPosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotSlot0Configs.kS = ArmConstants.armPosKS;
        pivotSlot0Configs.kV = ArmConstants.armPosKV;
        pivotSlot0Configs.kP = ArmConstants.armPosKP;
        pivotSlot0Configs.kD = ArmConstants.armPosKD;
        pivotSlot0Configs.kA = ArmConstants.armPosKA;
        
        var pivotMotionMagicConfigs = armPosConfigs.MotionMagic;
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.armPosVel;    //rps
        pivotMotionMagicConfigs.MotionMagicAcceleration = ArmConstants.armPosAccel;    //rps/s
        pivotMotionMagicConfigs.MotionMagicJerk = ArmConstants.armPosJerk;          //rps/s/s

       

        rotMotor.getConfigurator().apply(armPosConfigs);

        rotMotor.setPosition(0);
    }

    public void runArm(double speed) {
        DutyCycleOut armRequest = new DutyCycleOut(0.0);
        endEffectorMotor.setControl(armRequest.withOutput(speed));
    }
    public void rotArm(double pos) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        rotMotor.setControl(m_request.withPosition(pos));
    }

    public double getRot() {
        return armEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setRotState(armRotState state) {
        currentRotState = state;
    }

    public armRotState getRotState() {
        return currentRotState;
    }

    public void setIntakeState(armOuttakeState state) {
        currentOuttakeState = state;
    }

    public armRotState getIntakeState() {
        return currentRotState;
    }
    public void restArmPos() {
        armEncoder.setPosition(0);
    }
    public void logging() {
        SmartDashboard.putNumber("Arm Pos", getRot());
        SmartDashboard.putNumber("Arm Speed", rotMotor.get());
    }
    public void periodic() {
        logging();
        switch (currentOuttakeState) {
            case IDLE:
                runArm(0);
                break;
            case INTAKE:
                runArm(ArmConstants.armRunSpeed);
                break;
            case OUTTAKE:
                runArm(ArmConstants.armRunSpeed * -1);
                break;
        }
        switch(currentRotState) {
            case IDLE:
                rotArm(ArmConstants.armStowedPos);
                break;
            case FEEDPOS:
                rotArm(ArmConstants.armFeedPos);
                break;
            case LOWPOS:
                rotArm(ArmConstants.armLowPos);
                break;
            case MIDPOS:
                rotArm(ArmConstants.armMidPos);
                break;
            case HIGHPOS:
                rotArm(ArmConstants.armHighPos);
                break;

        }
    }
}
