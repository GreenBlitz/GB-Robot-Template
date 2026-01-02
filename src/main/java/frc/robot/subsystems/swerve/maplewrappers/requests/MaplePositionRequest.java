package frc.robot.subsystems.swerve.maplewrappers.requests;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;

public class MaplePositionRequest implements IRequest<Rotation2d> {

	private Rotation2d setPoint = new Rotation2d();

	@Override
	public IRequest<Rotation2d> withSetPoint(Rotation2d setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	@Override
	public Rotation2d getSetPoint() {
		return setPoint;
	}

}
