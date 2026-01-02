package frc.robot.subsystems.swerve.maplewrappers.requests;

import frc.robot.hardware.interfaces.IRequest;

public class MapleVoltageRequest implements IRequest<Double> {

	private Double setPoint = 0.0;

	@Override
	public IRequest<Double> withSetPoint(Double setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	@Override
	public Double getSetPoint() {
		return setPoint;
	}

}
