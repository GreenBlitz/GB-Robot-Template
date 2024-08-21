package frc.utils.visualizing;

public class Vector2D{
    protected double magnitude;
    protected double direction;
    protected VectorComponents vectorComponents;

    public Vector2D(double magnitude, double direction) {
        this.magnitude = magnitude;
        this.direction = direction;
        this.vectorComponents = new VectorComponents(cosDegrees(this.direction) * this.magnitude, sinDegrees(this.direction) * this.magnitude);
    }

    public Vector2D(VectorComponents vectorComponents) {
        this.magnitude = calculateMagnitude();
        this.direction = shiftTanDegrees();
        this.vectorComponents = new VectorComponents(vectorComponents.getHorizontal(), vectorComponents.getVertical());
    }

    public Vector2D(Vector2D other){
        this.magnitude = other.magnitude;
        this.direction = other.direction;
        this.vectorComponents = new VectorComponents(other.vectorComponents);
    }

    public void sumVectors(Vector2D other){
        this.addToHorizontal(other.vectorComponents.getHorizontal());
        this.addToVertical(other.vectorComponents.getVertical());
    }

    public void sumVectorsAndDivideByConstant(Vector2D other, double divisionConstant){
        this.addToHorizontal(other.vectorComponents.getHorizontal()
                /divisionConstant);
        this.addToVertical(other.vectorComponents.getVertical()
                /divisionConstant);
    }

    public void addToHorizontal(double addHorizontal) {
        this.vectorComponents.addToHorizontal(addHorizontal);
        this.magnitude = calculateMagnitude();
        this.direction = shiftTanDegrees();
    }

    public void addToVertical(double addVertical) {
        this.vectorComponents.addToVertical(addVertical);
        this.magnitude = calculateMagnitude();
        this.direction = shiftTanDegrees();
    }

    public void addToHorizontalAndVertical(double addHorizontal, double addVertical) {
        addToHorizontal(addHorizontal);
        addToVertical(addVertical);
    }

    private double sinDegrees(double angle) {
        return Math.sin(Math.toRadians(angle));
    }

    private double cosDegrees(double angle) {
        return Math.cos(Math.toRadians(angle));
    }

    private double shiftTanDegrees() {
        return Math.toDegrees(Math.atan(this.vectorComponents.getVertical() / this.vectorComponents.getHorizontal()));
    }

    private double calculateMagnitude() {
        return Math.sqrt(Math.pow(this.vectorComponents.getVertical(), 2) + Math.pow(this.vectorComponents.getHorizontal(), 2));
    }

    @Override
    public String toString() {
        return "Vector{" + "magnitude=" + magnitude + ", direction=" + direction + ", vectorComponents=" + vectorComponents + '}';
    }
}
