package frc.utils.math;

public class AngleMath {
    public static double closestAngle(double angle,double shouldBeClose){
        return angle+2*Math.PI*Math.ceil((shouldBeClose-Math.PI-angle)/360);
    }
}
