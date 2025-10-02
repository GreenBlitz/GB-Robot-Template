package frc.robot.hardware.phoenix6.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class TalonFXFollowerConfig {

	public String[] names;
	public Phoenix6DeviceID[] followerIDs;
	public boolean[] followerInvertedToMain;
	public TalonFXConfiguration followerConfig;

}
