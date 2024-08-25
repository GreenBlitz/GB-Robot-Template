package frc.utils.ObjectVisualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public abstract class VisualizerCommand extends Command {
    protected Timer timer;
    protected Pose3d position;
    protected Visualizer visualizer;

    public VisualizerCommand(){
        this.timer = new Timer();
        this.visualizer = new Visualizer();
        this.position = visualizer.currentPosition(0);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        position = visualizer.currentPosition(timer.get());
        Logger.recordOutput(
                "NoteVisualizer",
                new Pose3d[]{position}
        );
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Visualizer", new Pose3d[]{});
    }
}
