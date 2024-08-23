package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class arm extends GBSubsystem {

	IMotor arm;
	MotorInputsAutoLogged inputs;
	PositionTorqueCurrentFOC positionControl;

	public arm() {
		super("arm");
		inputs = new MotorInputsAutoLogged();
		positionControl = new PositionTorqueCurrentFOC(0).withSlot(0);
	}

	public void setTargetAngle(Rotation2d angle) {
		arm.setTargetAngle(
				new CloseLoopControl(positionControl.withPosition(angle.getRotations()), positionControl.Slot, angle, ControlState.PID)
		);
	}

	@Override
	protected void subsystemPeriodic() {
		arm.updateInputs(inputs);
		Logger.processInputs("arm", inputs);
	}

	public boolean isAtAngle() {
		return false;
	}

}
