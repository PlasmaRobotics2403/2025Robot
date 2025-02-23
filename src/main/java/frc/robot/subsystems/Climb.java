package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climb{
    private TalonFX climbMotor;

    public climbState currentState;
    public enum climbState{
        IDLE,
        CLIMBUP,
        CLIMBDOWN
    }

    public Climb() {
        climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);

        currentState = climbState.IDLE;

    }

    public void setState(climbState state) {
        currentState = state;
    }

    public climbState getState() {
        return currentState;
    }
    public void runClimb(double speed) {
        DutyCycleOut intakeRequest = new DutyCycleOut(0.0);
        climbMotor.setControl(intakeRequest.withOutput(speed));
    }

    public void periodic() {
        switch (currentState) {
            case IDLE:
                runClimb(0);
                break;
            case CLIMBUP:
                runClimb(ClimbConstants.CLIMB_SPEED);
                break;
            case CLIMBDOWN:
                runClimb(-ClimbConstants.CLIMB_SPEED);
                break;
        }
    }
}
