package frc.robot.hardware.phoenix6.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class TalonFXFollowerConfig {

	public record TalonFXFollowerID(String name, Phoenix6DeviceID id, boolean opposeMain){}

	public TalonFXFollowerID[] followerIDs;
	public TalonFXConfiguration motorConfig;

}
