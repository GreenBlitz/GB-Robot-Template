package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributionDeviceID {
    private int id;
    private PowerDistribution.ModuleType type;

    public PowerDistributionDeviceID(int id, PowerDistribution.ModuleType type){
        this.id = id;
        this.type = type;
    }

    public int getId() {
        return id;
    }

    public PowerDistribution.ModuleType getType() {
        return type;
    }

    public PowerDistributionDeviceID withID(int id){
        this.id = id;
        return this;
    }

    public PowerDistributionDeviceID withType(PowerDistribution.ModuleType type){
        this.type = type;
        return this;
    }
}
