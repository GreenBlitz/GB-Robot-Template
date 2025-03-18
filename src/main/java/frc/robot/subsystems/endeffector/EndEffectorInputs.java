package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class EndEffectorInputs {

	public EndEffectorData data = new EndEffectorData(0, 0, false, false);

	public record EndEffectorData(double power, double current, boolean isCoralIn, boolean isAlgaeIn) {}

}
