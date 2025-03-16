package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ArmInputs {

	public ArmIOData data = new ArmIOData(0, 0, 0, 0, 0);

	public record ArmIOData(
		double positionRads,
		double voltage,
		double encoderPositionRads,
		double targetPositionRads,
		double reversedSoftLimitRads
	) {}

}
