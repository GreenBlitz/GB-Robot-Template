package frc.robot.hardware.phoenix6.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class TalonFXFollowerConfig {

	public final TalonFXFollowerID[] followerIDs = new TalonFXFollowerID[0];
	public final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

	public record TalonFXFollowerID(String name, Phoenix6DeviceID id, boolean opposeMain) {}

}
