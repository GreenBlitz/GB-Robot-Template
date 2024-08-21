package frc.utils.ObjectVisualizer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class Visualizer {
    Vector<N3> acceleration;
    Vector<N3> velocity;
    Vector<N3> location;

    public Visualizer(Vector<N3> acceleration, Vector<N3> velocity){
        this.acceleration = new Vector<N3>(acceleration);
        this.velocity = new Vector<N3>(velocity);
        this.location = VecBuilder.fill(0,0,0);
    }
    public Visualizer(Vector<N3> acceleration, Vector<N3> velocity, Vector<N3> location){
        this.acceleration = new Vector<N3>(acceleration);
        this.velocity = new Vector<N3>(velocity);
        this.location = new Vector<N3>(location);
    }
    public Translation3d currentPosition(double time){
        location.plus(velocity.div(1/time);
        location.plus(acceleration.div(2/(time*time)));
        return new Translation3DRan(location.getData());
    }
}
