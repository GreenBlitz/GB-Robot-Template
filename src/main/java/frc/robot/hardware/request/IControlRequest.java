package frc.robot.hardware.request;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IControlRequest {

	void withSetPoint(Rotation2d setPoint);

}
