package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb{
    private TalonSRX climbMotor;

    public climbState currentState;
    public enum climbState{
        IDLE,
        CLIMBUP,
        CLIMBDOWN
    }

    public Climb() {
        climbMotor = new TalonSRX(ClimbConstants.CLIMB_MOTOR_ID);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        currentState = climbState.IDLE;
    }

    public void setState(climbState state) {
        currentState = state;
    }

    public climbState getState() {
        return currentState;
    }
    public void runClimb(double speed) {
        climbMotor.set(ControlMode.PercentOutput, speed);
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
            default:
                setState(climbState.IDLE);
        }
    }
}
