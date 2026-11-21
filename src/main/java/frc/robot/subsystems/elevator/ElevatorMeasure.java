package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class ElevatorMeasure  {
    public final double value;
    public final MeasureType measureType;
    public enum MeasureType{
        PHYSICAL_DISTANCE_CM,
        FRACTIONAL_DISTANCE;
    }
    private ElevatorMeasure(double value){
        this.value = value;
        this.measureType=MeasureType.FRACTIONAL_DISTANCE;
    }
    private ElevatorMeasure(Distance distance){
        this.value = distance.in(Units.Centimeter);
        this.measureType=MeasureType.PHYSICAL_DISTANCE_CM;
    }
    public ElevatorMeasure createFromFractional(double fractional){
        return new ElevatorMeasure(fractional);
    }
    public ElevatorMeasure createFromHeight(Distance height){
        return new ElevatorMeasure(height);
    }
}
