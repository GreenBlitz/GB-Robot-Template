package frc.utils.ObjectVisualizer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class VisualizerCommand extends Command {
    protected Timer timer;
    protected Pose3d position;
    protected Visualizer visualizer;

    public VisualizerCommand(Vector<N3> acceleration, Vector<N3> velocity, Vector<N3> location) {
        this.timer = new Timer();
        this.visualizer = new Visualizer(
                new Vector<N3>(acceleration),
                new Vector<N3>(velocity),
                new Vector<N3>(location)
        );
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
//        visualizer.resetLocation();
        position = visualizer.currentPosition(timer.get());
        SmartDashboard.putString("Position-Note", position.toString());
        Logger.recordOutput(
                "NoteVisualizer",
                new Pose3d[]{position}

        );
        System.out.println(position.toString());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Visualizer", new Pose3d[]{});
    }
}
