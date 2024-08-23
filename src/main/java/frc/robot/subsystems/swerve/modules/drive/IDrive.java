package frc.robot.subsystems.swerve.modules.drive;

import frc.robot.hardware.IMotor;

public interface IDrive extends IMotor {

	void updateInputs(DriveThreadMetersInputsAutoLogged inputs);

}
