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
	CloseLoopControl positionControl;


	public roller() {
		super("roller");
		inputs = new MotorInputsAutoLogged();
		PositionTorqueCurrentFOC a = new PositionTorqueCurrentFOC(0).withSlot(1);
		positionControl = new CloseLoopControl(a, 0, () -> Rotation2d.fromRotations(a.Position), ControlState.PID);
	}

	public void setTargetAngle(Rotation2d angle) {
		roller.setTargetAngle(positionControl.withPosition()); //use angle
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
