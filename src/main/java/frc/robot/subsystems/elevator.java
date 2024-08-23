package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class elevator extends GBSubsystem {

	IMotor elevator;
	MotorInputsAutoLogged motorInputs;
	MetersInputsAutoLogged metersInputs;
    PositionTorqueCurrentFOC positionControl;


	public elevator() {
		super("arm");
		motorInputs = new MotorInputsAutoLogged();
        positionControl = new PositionTorqueCurrentFOC(0).withSlot(1);
	}

	double angleToMeters(Rotation2d rotation2d) {
		return 1;
	}

	public void setTargetAngle(Rotation2d angle) {
		elevator.setTargetAngle(
			new CloseLoopControl(positionControl.withPosition(angle.getRotations()), positionControl.Slot, angle, ControlState.PID)
		);
	}

	@Override
	protected void subsystemPeriodic() {
		elevator.updateInputs(motorInputs);
		Logger.processInputs("arm", motorInputs);
	}

	public boolean isAtAngle() {
		return false;
	}

}
