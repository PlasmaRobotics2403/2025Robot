package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class Arm {
    private TalonFX rotMotor;
    private TalonFX endEffectorMotor;
    private CANcoder armEncoder;
    private boolean limitState;
    final MotionMagicVoltage rotArmRequest;
    final MotionMagicVoltage rotArmBackwardRequest;

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
        LEVELTHREE,
        HIGHPOS,
    }
    public Arm() {

        rotMotor = new TalonFX(ArmConstants.armRotID);
        endEffectorMotor = new TalonFX(ArmConstants.armOuttakeID);
        armEncoder = new CANcoder(ArmConstants.armCancoderID, "rio");

        currentOuttakeState = armOuttakeState.IDLE;
        currentRotState = armRotState.IDLE;
        rotArmRequest = new MotionMagicVoltage(0);
        rotArmBackwardRequest = new MotionMagicVoltage(0);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.MagnetOffset = 0.54;
        
        armEncoder.getConfigurator().apply(encoderConfig);

        final TalonFXConfiguration armPosConfigs = new TalonFXConfiguration();
        armPosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        armPosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        armPosConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armPosConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        armPosConfigs.Feedback.FeedbackRemoteSensorID = ArmConstants.armCancoderID;
        armPosConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        var pivotSlot0Configs = armPosConfigs.Slot0;

        armPosConfigs.CurrentLimits.StatorCurrentLimit = 40;
        armPosConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotSlot0Configs.kS = ArmConstants.armPos0KS;
        pivotSlot0Configs.kV = ArmConstants.armPos0KV;
        pivotSlot0Configs.kP = ArmConstants.armPos0KP;
        pivotSlot0Configs.kD = ArmConstants.armPos0KD;
        pivotSlot0Configs.kA = ArmConstants.armPos0KA;
        
        pivotSlot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        var pivotSlot1Configs = armPosConfigs.Slot1;
        pivotSlot1Configs.kS = ArmConstants.armPos1KS;
        pivotSlot1Configs.kV = ArmConstants.armPos1KV;
        pivotSlot1Configs.kP = ArmConstants.armPos1KP;
        pivotSlot1Configs.kD = ArmConstants.armPos1KD;
        pivotSlot1Configs.kA = ArmConstants.armPos1KA;
        
        pivotSlot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        
        var pivotMotionMagicConfigs = armPosConfigs.MotionMagic;
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.armPosVel*ArmConstants.armGearRatio;    //rps
        pivotMotionMagicConfigs.MotionMagicAcceleration = ArmConstants.armPosAccel*ArmConstants.armGearRatio;    //rps/s
        //pivotMotionMagicConfigs.MotionMagicJerk = ArmConstants.armPosJerk;          //rps/s/s

       

        rotMotor.getConfigurator().apply(armPosConfigs);

        rotMotor.setPosition(0);
    }

    public void runArm(double speed) {
        DutyCycleOut armRequest = new DutyCycleOut(0.0);
        endEffectorMotor.setControl(armRequest.withOutput(speed));
    }

    public void rotArm(double pos) {
        rotArmRequest.Slot = 1;
        double gravityAssist = ArmConstants.armGravityK * Math.sin(2*Math.PI*(getRot()-0.25));
        SmartDashboard.putNumber("Arm Commanded Pos", pos);
        rotMotor.setControl(rotArmRequest.withPosition(pos).withFeedForward(gravityAssist));
    }
    public void rotArmBackward(double pos) {
        rotArmBackwardRequest.Slot = 0;
        double gravityAssist = ArmConstants.armGravityK * Math.sin(2*Math.PI*(getRot()-0.25));
        rotMotor.setControl(rotArmBackwardRequest
                .withPosition(pos)
                .withLimitForwardMotion(limitState)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
                .withLimitReverseMotion(false)
                .withFeedForward(gravityAssist));
        //DriverStation.reportWarning("MOVING ARM", false);
        SmartDashboard.putNumber("Arm Commanded Pos", pos);
    }
   
    public void setLimitMotion(boolean input) {
        limitState = input;
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
        SmartDashboard.putBoolean("Intake Virtual Limit Switch", limitState);
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
                rotArm(ArmConstants.armMidPosForward);
                break;
            case LEVELTHREE:
                rotArmBackward(ArmConstants.armMidPos);
                break;
            case HIGHPOS:
                rotArmBackward(ArmConstants.armHighPos);
                break;

        }
    }
}
