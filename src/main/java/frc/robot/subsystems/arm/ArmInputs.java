package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ArmInputs {

	public ArmData data = new ArmData(0, 0, 0, 0, 0);

	public record ArmData(
		double positionRads,
		double voltage,
		double encoderPositionRads,
		double targetPositionRads,
		double reversedSoftLimitRads
	) {}

}
