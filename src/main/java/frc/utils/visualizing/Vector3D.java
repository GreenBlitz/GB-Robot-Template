package frc.utils.visualizing;

public class Vector3D {

    protected double magnitude;
    protected double chassisAngle;
    protected double pivotAngle;
    protected VectorComponents vectorComponents;

    public Vector3D(double magnitude, double chassisAngle, double pivotAngle) {
        this.magnitude = magnitude;
        this.chassisAngle = chassisAngle;
        this.pivotAngle = pivotAngle;
        this.vectorComponents = new VectorComponents(
                this.magnitude * cosDegrees(this.chassisAngle)
                        * cosDegrees(this.pivotAngle),
                this.magnitude * sinDegrees(this.pivotAngle),
                this.magnitude * cosDegrees(this.pivotAngle) *
                        sinDegrees(this.chassisAngle)
        );
    }

    public Vector3D(VectorComponents vectorComponents){
        this.magnitude = this.calculateMagnitude(vectorComponents);
        this.chassisAngle = shiftTanDegrees()
    }

    private double sinDegrees(double angle) {
        return Math.sin(Math.toRadians(angle));
    }

    private double cosDegrees(double angle) {
        return Math.cos(Math.toRadians(angle));
    }

    private double shiftTanDegrees(double horizontal, double vertical) {
        return Math.toDegrees(Math.atan(vertical / horizontal));
    }
    private double calculateMagnitude(VectorComponents vectorComponents){
        return Math.sqrt(Math.pow(vectorComponents.getHorizontal(),2)
                +Math.pow(vectorComponents.getVertical(),2)
                +Math.pow(vectorComponents.getInner(),2));
    }
}
