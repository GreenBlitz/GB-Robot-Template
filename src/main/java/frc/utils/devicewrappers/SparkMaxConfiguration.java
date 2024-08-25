package frc.utils.devicewrappers;

import com.ctre.phoenix6.configs.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxConfiguration {

    public double conversionFactor;
    public Rotation2d forwardAngleLimit;
    public Rotation2d backwardAngleLimit;
    public boolean enableSoftLimit;

    public SparkMaxConfiguration(){

    }

    public SparkMaxConfiguration withConversionFactor(double newConversionFactor){
        this.conversionFactor = newConversionFactor;
        return this;
    }

    public SparkMaxConfiguration withForwardAngleLimit(Rotation2d newForwardAngleLimit){
        this.forwardAngleLimit = newForwardAngleLimit;
        return this;
    }

    public SparkMaxConfiguration withBackwardAngleLimit(Rotation2d newBackwardAngleLimit){
        this.backwardAngleLimit = newBackwardAngleLimit;
        return this;
    }

    public SparkMaxConfiguration withEnableSoftLimit(boolean newEnableSoftLimit){
        this.enableSoftLimit = newEnableSoftLimit;
        return this;
    }



}
