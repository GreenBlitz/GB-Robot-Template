package frc.utils.visualizing;

public class VectorComponents {
    private double horizontal;
    private double vertical;

    public VectorComponents(double horizontal, double vertical) {
        this.horizontal = horizontal;
        this.vertical = vertical;
    }
    public VectorComponents(VectorComponents other){
        this.horizontal = other.getHorizontal();
        this.vertical = other.getVertical();
    }

    public void setHorizontal(double horizontal) {
        this.horizontal = horizontal;
    }

    public void setVertical(double vertical) {
        this.vertical = vertical;
    }

    public double getHorizontal() {
        return horizontal;
    }

    public double getVertical() {
        return vertical;
    }
    public void addToHorizontal(double addHorizontal){
        setHorizontal(addHorizontal+this.horizontal);
    }
    public void addToVertical(double addVertical){
        setVertical(addVertical+this.vertical);
    }
    public void addToHorizontalAndVertical(double addHorizontal, double addVertical){
        addToHorizontal(addHorizontal);
        addToVertical(addVertical);
    }

    @Override
    public String toString() {
        return "VectorComponents{" +
                "horizontal=" + horizontal +
                ", vertical=" + vertical +
                '}';
    }
}
