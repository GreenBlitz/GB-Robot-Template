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
	CloseLoopControl positionControl;
	CloseLoopControl climbingPositionControl;

	public climber() {
		super("climber");
		PositionTorqueCurrentFOC a = new PositionTorqueCurrentFOC(0).withSlot(0);
		PositionTorqueCurrentFOC b = new PositionTorqueCurrentFOC(0).withSlot(1);
		positionControl = new CloseLoopControl(a, 0, () -> Rotation2d.fromRotations(a.Position), ControlState.PID);
		climbingPositionControl = new CloseLoopControl(b, 1, () -> Rotation2d.fromRotations(a.Position), ControlState.PID);
		motorInputs = new MotorInputsAutoLogged();
	}

	double angleToMeters(Rotation2d rotation2d) {
		return 1;
	}

	public void setTargetAngle(Rotation2d angle) {
		climber.setTargetAngle(positionControl.withPosition());
	}

	public void setClimbingAngleTargetAngle(Rotation2d angle) {
		climber.setTargetAngle(climbingPositionControl.withPosition());
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
