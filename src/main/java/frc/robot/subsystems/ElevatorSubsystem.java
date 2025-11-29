package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
	private TalonFX elevatorMotorMaster = new TalonFX(Constants.IDs.ELEVATOR_MOTOR_MASTER_ID); // left motor
	private TalonFX elevatorMotorFollow = new TalonFX(Constants.IDs.ELEVATOR_MOTOR_FOLLOW_ID); // right motor

	private PositionVoltage elevatorPositionVoltage = new PositionVoltage(null);



	/* Creates a new ShooterSubsystem */
	public ElevatorSubsystem () {}

	public double getElevatorPos() {
		return elevatorPos;
	}

	public void elevatorRunVelocity (double velocity) {
		leftMotorDutyCycle.Velocity = velocity;
		elevatorMotorLeft.setControl(leftMotorDutyCycle);
		// elevatorMotorLeft.set(
		// 	TalonSRXControlMode.PercentOutput,
		// 	0
		// );
		
	}

	@Override
	public void periodic () {
		System.out.println("elevator position" + elevatorPos);
	}
	
}
