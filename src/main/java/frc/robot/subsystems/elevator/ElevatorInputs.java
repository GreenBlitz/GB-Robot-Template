package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {

	public ElevatorIOData data = new ElevatorIOData(0, 0, 0, 0, 0, 0);

	public record ElevatorIOData(
		double leftPositionRads,
		double leftVoltage,
		double rightPositionRads,
		double rightVoltage,
		double positionMeters,
		double targetPositionMeters
	) {}

}
