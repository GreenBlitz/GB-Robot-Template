package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;

public record PowerDistributionDeviceID(int ID, PowerDistribution.ModuleType type) {}
