package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;

public record PowerDistributionDeviceID(int id, PowerDistribution.ModuleType type) {}
