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
	CloseLoopControl positionControl;

	public arm() {
		super("arm");
		inputs = new MotorInputsAutoLogged();
		PositionTorqueCurrentFOC a = new PositionTorqueCurrentFOC(0).withSlot(0);
		positionControl = new CloseLoopControl(a, 0, () -> Rotation2d.fromRotations(a.Position), ControlState.PID);
	}

	public void setTargetAngle(Rotation2d angle) {
		arm.setTargetAngle(positionControl.withPosition());
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
