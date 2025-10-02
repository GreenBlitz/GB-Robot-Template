package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {

	public ElevatorData data = new ElevatorData(0, 0, 0, 0);

	public record ElevatorData(double positionRads, double voltage, double positionMeters, double targetPositionMeters) {}

}
