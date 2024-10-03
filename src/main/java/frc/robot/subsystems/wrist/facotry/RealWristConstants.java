package frc.robot.subsystems.wrist.facotry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.talonsrx.TalonSRXMotor;
import frc.robot.hardware.request.srx.AngleSRXRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.wrist.WristStuff;
import frc.utils.AngleUnit;

public class RealWristConstants {
	

	
	protected static WristStuff generateWristStuff(String logPath){
		TalonSRX motor = new TalonSRX(IDs.TalonSRXs.WRIST);
		motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		
		return new WristStuff(
				logPath,
				new TalonSRXMotor(logPath, motor, Rotation2d.fromRotations(2)),
				new AngleSRXRequest(ControlMode.Position,0),
				new SuppliedAngleSignal("position", () -> motor.getSelectedSensorPosition(), AngleUnit.ROTATIONS)
		);
		
	}
	
	
	
	
	
}
