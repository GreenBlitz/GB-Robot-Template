package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {

	public ElevatorData data = new ElevatorData(0, 0, 0, 0, 0, 0);

	public record ElevatorData(
		double leftPositionRads,
		double leftVoltage,
		double rightPositionRads,
		double rightVoltage,
		double positionMeters,
		double targetPositionMeters
	) {}

}
