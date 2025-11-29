package frc.robot.commands;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRunCommand extends Command {
	private ElevatorSubsystem m_elevatorSubsystem;
	private TalonFX elevatorMotorMaster = new TalonFX(Constants.IDs.ELEVATOR_MOTOR_MASTER_ID); // left motor
	private TalonFX elevatorMotorFollow = new TalonFX(Constants.IDs.ELEVATOR_MOTOR_FOLLOW_ID); // right motor

	private PositionVoltage elevatorPositionVoltage = new PositionVoltage(null);
	
	private String level;
	private double elevatorPos = 0;

	private boolean isKilled = false;

	private VelocityDutyCycle leftMotorDutyCycle = new VelocityDutyCycle(null);
	private Follower rightMotFollower = new Follower(0, false);

	public ElevatorRunCommand (String levelParam, ElevatorSubsystem elevatorSubsystem) {
		m_elevatorSubsystem = elevatorSubsystem;
		addRequirements(elevatorSubsystem);
		this.level = levelParam;
	}

	public void elevatorMoveToLevel (String level) {
		if (level == "l1") {
			m_elevatorSubsystem.elevatorRunVelocity(Constants.ElevatorConstants.L1_VELOCITY);
		} else if (level == "l2") {
			m_elevatorSubsystem.elevatorRunVelocity(Constants.ElevatorConstants.L2_VELOCITY);
		} else if (level == "l3") {
			m_elevatorSubsystem.elevatorRunVelocity(Constants.ElevatorConstants.L3_VELOCITY);
		} else if (level == "l4") {
			m_elevatorSubsystem.elevatorRunVelocity(Constants.ElevatorConstants.L4_VELOCITY);
		} else if (level == "barge") {
			m_elevatorSubsystem.elevatorRunVelocity(Constants.ElevatorConstants.BARGE_VELOCITY);
		} else if (level == "source") {
			m_elevatorSubsystem.elevatorRunVelocity(Constants.ElevatorConstants.SOURCE_VELOCITY);
		}
	}

	@Override
	public void initialize () {
		System.out.println("elevator command triggered. moving to level: "+ level);
	}

	@Override
	public void execute () {
		elevatorMoveToLevel(level);
	}

	@Override
	public void end (boolean interrupted) {
		System.out.println("elevator has reached target level ("+level+")");
		
	}

	@Override
	public boolean isFinished () {
		return isKilled;
	}




}
