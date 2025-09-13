package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.hardware.mechanisms.MechanismSimulation;

public class TalonFXFollowerConfig {
    public Phoenix6DeviceID[] followerIDs = new Phoenix6DeviceID[0];
    public MechanismSimulation[] mechanismSimulations = new MechanismSimulation[0];
    public BusChain[] followerBuses = new BusChain[0];
    public TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    public boolean[] followerOpposeMain = new boolean[0];
}
