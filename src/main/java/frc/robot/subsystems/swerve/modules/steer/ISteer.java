package frc.robot.subsystems.swerve.modules.steer;

import frc.robot.hardware.IMotor;

public interface ISteer extends IMotor {

	void updateInputs(SteerThreadInputsAutoLogged inputs);

}
