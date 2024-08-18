package frc.utils.visualizing;

public class VectorComponents {
    private double horizontal;
    private double vertical;

    private double inner;

    public VectorComponents(double horizontal, double vertical, double inner) {
        this.horizontal = horizontal;
        this.vertical = vertical;
        this.inner = inner;
    }
    public VectorComponents(VectorComponents other){
        this.horizontal = other.getHorizontal();
        this.vertical = other.getVertical();
        this.inner = other.getInner();
    }

    public void setHorizontal(double horizontal) {
        this.horizontal = horizontal;
    }

    public void setVertical(double vertical) {
        this.vertical = vertical;
    }
    public void setInner(double inner){
        this.inner = inner;
    }

    public double getHorizontal() {
        return horizontal;
    }

    public double getVertical() {
        return vertical;
    }
    public double getInner(){
        return inner;
    }
    public void addToHorizontal(double addHorizontal){
        setHorizontal(addHorizontal+this.horizontal);
    }
    public void addToVertical(double addVertical){
        setVertical(addVertical+this.vertical);
    }
    public void addToInner(double addInner){
        setInner(addInner+this.inner);
    }
    public void addToHorizontalAndVertical(double addHorizontal, double addVertical, double addInner){
        addToHorizontal(addHorizontal);
        addToVertical(addVertical);
        addToInner(addInner);
    }

    @Override
    public String toString() {
        return "VectorComponents{" +
                "horizontal=" + horizontal +
                ", vertical=" + vertical +
                '}';
    }
}
