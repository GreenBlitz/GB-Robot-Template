package frc.robot.hardware.phoenix6.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class TalonFXFollowerConfig {

	public TalonFXFollowerID[] followerIDs = new TalonFXFollowerID[0];
	public TalonFXConfiguration motorConfig = new TalonFXConfiguration();

	public record TalonFXFollowerID(String name, Phoenix6DeviceID id, MotorAlignmentValue motorAlignmentValue) {}

}
