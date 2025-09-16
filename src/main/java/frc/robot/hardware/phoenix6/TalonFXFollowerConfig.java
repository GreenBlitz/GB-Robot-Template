package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.hardware.mechanisms.MechanismSimulation;

public class TalonFXFollowerConfig {



    public Phoenix6DeviceID[] followerIDs;
    public MechanismSimulation[] mechanismSimulations;
    public BusChain[] followerBuses;
    public TalonFXConfiguration followerConfig;
    public boolean[] followerOpposeMain;
}
