package frc.robot.subsystems.wrist.facotry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.talonsrx.TalonSRXMotor;
import frc.robot.hardware.request.srx.AngleSRXRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristStuff;
import frc.utils.AngleUnit;
import frc.utils.Conversions;

public class RealWristConstants {

	private static final int POSITION_PID_SLOT = 0;

	protected static WristStuff generateWristStuff(String logPath) {
		TalonSRX motor = new TalonSRX(IDs.TalonSRXs.WRIST);
		motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		motor.config_kP(POSITION_PID_SLOT, 1);

		return new WristStuff(
			logPath,
			new TalonSRXMotor(logPath, motor, WristConstants.GEAR_RATIO),
			new AngleSRXRequest(ControlMode.Position, POSITION_PID_SLOT),
			new SuppliedAngleSignal(
				"position",
				() -> Conversions.magTicksToAngle(motor.getSelectedSensorPosition(), 2).getRotations(),
				AngleUnit.ROTATIONS
			)
		);
	}

}
