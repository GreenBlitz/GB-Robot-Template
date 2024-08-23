package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class roller extends GBSubsystem {

	IMotor roller;
	MotorInputsAutoLogged inputs;
	PositionTorqueCurrentFOC positionControl;


	public roller() {
		super("roller");
		inputs = new MotorInputsAutoLogged();
		positionControl = new PositionTorqueCurrentFOC(0).withSlot(1);
	}

	public void setTargetAngle(Rotation2d angle) {
		roller.setTargetAngle(
			new CloseLoopControl(positionControl.withPosition(angle.getRotations()), positionControl.Slot, angle, ControlState.PID)
		);
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
