package frc.utils.visualizing.Kinematics;

import utils.Vector;

public class Location extends Vector {
    public Location(double location, double direction) {
        super(location, direction);
    }

    public void acceleratedlyEqualMotion(Velocity velocity) {
        super.sumVectorsAndDivideByConstant(velocity, KinimaticsConstants.ITERATIONS_IN_A_SECOND);
    }
    public double getX(){
        return this.vectorComponents.getHorizontal();
    }
    public double getY(){
        return this.vectorComponents.getVertical();
    }
}
