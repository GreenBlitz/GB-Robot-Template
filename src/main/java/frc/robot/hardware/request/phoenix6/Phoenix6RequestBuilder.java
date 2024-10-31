package frc.robot.hardware.request.phoenix6;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;

public class Phoenix6RequestBuilder {

	public Phoenix6Request<Rotation2d> generate(PositionVoltage positionVoltage) {
		return new Phoenix6Request<>(positionVoltage, setPoint -> positionVoltage.withPosition(setPoint.getRotations()));
	}

	public Phoenix6Request<Rotation2d> generate(VelocityVoltage velocityVoltage) {
		return new Phoenix6Request<>(velocityVoltage, setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()));
	}

	public Phoenix6Request<Double> generate(VoltageOut voltageOut) {
		return new Phoenix6Request<>(voltageOut, voltageOut::withOutput);
	}

	public Phoenix6Request<Double> generate(TorqueCurrentFOC torqueCurrentFOC) {
		return new Phoenix6Request<>(torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

}
