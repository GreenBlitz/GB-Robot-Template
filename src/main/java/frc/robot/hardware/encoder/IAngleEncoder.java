package frc.robot.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;

public interface IAngleEncoder {

	void setPosition(Rotation2d position);

	boolean isConnected();

	void updateInputs(ConnectedInputAutoLogged inputs);

}
