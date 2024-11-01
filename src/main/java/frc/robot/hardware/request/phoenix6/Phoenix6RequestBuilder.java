package frc.robot.hardware.request.phoenix6;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;

public class Phoenix6RequestBuilder {

	public static Phoenix6Request<Rotation2d> build(PositionVoltage positionVoltage) {
		return new Phoenix6Request<>(new Rotation2d() ,positionVoltage, setPoint -> positionVoltage.withPosition(setPoint.getRotations()));
	}

	public static Phoenix6Request<Rotation2d> build(VelocityVoltage velocityVoltage) {
		return new Phoenix6Request<>(new Rotation2d() , velocityVoltage, setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()));
	}

	public static Phoenix6Request<Double> build(VoltageOut voltageOut) {
		return new Phoenix6Request<>(0.0, voltageOut, voltageOut::withOutput);
	}

	public static Phoenix6Request<Double> build(TorqueCurrentFOC torqueCurrentFOC) {
		return new Phoenix6Request<>(0.0, torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

}
