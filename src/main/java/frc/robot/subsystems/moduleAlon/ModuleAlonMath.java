package frc.robot.subsystems.moduleAlon;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleAlonMath {
    public static Rotation2d getSteerTargetAngle(Rotation2d targetAngle, Rotation2d currentAngle){
        return Rotation2d.fromRotations(
                getModInRange(targetAngle.getRotations(),currentAngle.getRotations()-0.25,currentAngle.getRotations()+0.25)
        );
    }
    public static boolean shouldFlipLinear(Rotation2d targetSteerAngle, Rotation2d currentSteerAngle){
        return Math.abs((targetSteerAngle.getRotations()-currentSteerAngle.getRotations())%0.5)>0.25;
    }
    public static double getModInRange(double x, double rangeStart, double rangeEnd){
        return rangeStart+modulu(x-rangeStart,rangeEnd-rangeStart);
    }
    public static double modulu(double x, double y){
        return ((x%y)+y)%y;
    }

}
