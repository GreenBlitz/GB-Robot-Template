package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class roller extends GBSubsystem {

	IMotor roller;
	MotorInputsAutoLogged inputs;

	public roller() {
		super("roller");
		inputs = new MotorInputsAutoLogged();
	}

	public void setTargetAngle(Rotation2d angle) {
		roller.setTargetAngle(angle, ControlState.PID);
	}

	@Override
	protected void subsystemPeriodic() {
		roller.updateInputs(inputs);
		Logger.processInputs("roller", inputs);
	}

	public boolean isAtAngle() {
		return false;
	}

}
