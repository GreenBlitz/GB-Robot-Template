package frc.robot.hardware.request.angle;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IAngleRequest {

	void withSetPoint(Rotation2d setPoint);

}
