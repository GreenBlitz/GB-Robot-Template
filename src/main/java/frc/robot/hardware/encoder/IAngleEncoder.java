package frc.robot.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.signal.InputSignal;

public interface IAngleEncoder {

	void setPosition(Rotation2d position);

	boolean isOK();

	void updateInputs(ConnectedInputAutoLogged inputs);

	void updateSignals(InputSignal... signal);

}
