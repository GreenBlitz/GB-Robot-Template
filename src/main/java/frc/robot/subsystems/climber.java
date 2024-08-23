package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class climber extends GBSubsystem {

	int CLIMBING_SLOT = 1;
	IMotor climber;
	MotorInputsAutoLogged motorInputs;
	MetersInputsAutoLogged metersInputs;
	PositionTorqueCurrentFOC positionControl;
	PositionTorqueCurrentFOC climbingPositionControl;

	public climber() {
		super("climber");
		positionControl = new PositionTorqueCurrentFOC(0).withSlot(0);
		climbingPositionControl = new PositionTorqueCurrentFOC(0).withSlot(1);
		motorInputs = new MotorInputsAutoLogged();
	}

	double angleToMeters(Rotation2d rotation2d) {
		return 1;
	}

	public void setTargetAngle(Rotation2d angle) {
		climber.setTargetAngle(
			new CloseLoopControl(positionControl.withPosition(angle.getRotations()), positionControl.Slot, angle, ControlState.PID)
		);
	}

	public void setClimbingAngleTargetAngle(Rotation2d angle) {
		climber.setTargetAngle(
			new CloseLoopControl(positionControl.withPosition(angle.getRotations()), positionControl.Slot, angle, ControlState.PID)
		);
	}

	@Override
	protected void subsystemPeriodic() {
		climber.updateInputs(motorInputs);
		Logger.processInputs("arm", motorInputs);
	}

	public boolean isAtAngle() {
		return false;
	}

}
