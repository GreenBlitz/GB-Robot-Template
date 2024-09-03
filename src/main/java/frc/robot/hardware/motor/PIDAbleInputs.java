package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PIDAbleInputs { //TODO: rename to a name about the info and not the control

	public Rotation2d position = new Rotation2d();
	public Rotation2d velocity = new Rotation2d();
	public Rotation2d acceleration = new Rotation2d();

}
