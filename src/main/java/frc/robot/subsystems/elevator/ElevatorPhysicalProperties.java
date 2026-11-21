package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public record ElevatorPhysicalProperties(Rotation2d motorMinPosition, Rotation2d motorMaxPosition, Distance elevatorHeight) {
    public Rotation2d convertFractionalHeightToMotorPosition(double fractionalHeight){
        return motorMaxPosition.times(fractionalHeight).plus(motorMinPosition.times(1-fractionalHeight));
    }
    public double convertMotorPositionToFractionalHeight(Rotation2d motorPosition){
        return motorPosition.minus(motorPosition).getDegrees()/motorMaxPosition.minus(motorMinPosition).getDegrees();
    }
    public double convertPhysicalHeightToFractionalHeight(Distance physicalHeight){
        return physicalHeight.div(elevatorHeight).magnitude();
    }
    public Distance convertFractionalHeightToPhysicalHeight(double fractionalHeight){
        return elevatorHeight.times(fractionalHeight);
    }
    public Rotation2d convertPhysicalHeightToMotorPosition(Distance physicalHeight){
        return convertFractionalHeightToMotorPosition(convertPhysicalHeightToFractionalHeight(physicalHeight));
    }
    public Distance convertMotorPositionToPhysicalHeight(Rotation2d motorPosition){
        return convertFractionalHeightToPhysicalHeight(convertMotorPositionToFractionalHeight(motorPosition));
    }
    public double getFractionalHeight(ElevatorMeasure measure){
        return switch (measure.measureType){
            case FRACTIONAL_DISTANCE -> measure.value;
            case PHYSICAL_DISTANCE_CM -> convertPhysicalHeightToFractionalHeight(Units.Centimeter.of(measure.value));
        };
    }

}
