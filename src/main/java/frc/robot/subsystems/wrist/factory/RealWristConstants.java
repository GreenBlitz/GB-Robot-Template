package frc.robot.subsystems.wrist.factory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.talonsrx.TalonSRXMotor;
import frc.robot.hardware.request.srx.AngleSRXRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristStuff;
import frc.utils.AngleUnit;
import frc.utils.Conversions;

import java.util.function.Supplier;

public class RealWristConstants {

	private static final int POSITION_PID_SLOT = 0;

	private static void configMotor(TalonSRX motor) {
		motor.configFactoryDefault();

		motor.setInverted(false);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.configPeakCurrentLimit(30);
		motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		motor.config_kP(POSITION_PID_SLOT, 1);
	}

	protected static WristStuff generateWristStuff(String logPath) {
		TalonSRX motor = new TalonSRX(IDs.TalonSRXs.WRIST);
		configMotor(motor);

		Supplier<Double> positionSupplier = () -> Conversions.magTicksToAngle(motor.getSelectedSensorPosition(), WristConstants.GEAR_RATIO)
			.getRotations();
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", positionSupplier, AngleUnit.ROTATIONS);

		return new WristStuff(
			logPath,
			new TalonSRXMotor(logPath, motor, WristConstants.GEAR_RATIO),
			new AngleSRXRequest(ControlMode.Position, POSITION_PID_SLOT),
			positionSignal
		);
	}

}
