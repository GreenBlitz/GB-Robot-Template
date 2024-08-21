package frc.utils.visualizing.Kinematics;

import frc.utils.visualizing.Vector2D;

public class Velocity extends Vector2D {

    public Velocity(double velocity, double direction) {
        super(velocity, direction);
    }

    public void accelerate(Acceleration acceleration) {
        super.sumVectorsAndDivideByConstant(acceleration, KinimaticsConstants.ITERATIONS_IN_A_SECOND);
    }
}
