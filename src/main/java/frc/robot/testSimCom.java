package frc.robot;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class testSimCom extends Command {

    private final testSim sim;

    private Rotation2d target;

    public testSimCom(testSim sim, Rotation2d target) {
        this.sim = sim;
        this.target = target;
    }

    @Override
    public void initialize() {
        sim.resetPosition(Rotation2d.fromDegrees(45));
    }

    @Override
    public void execute() {
        PositionVoltage positionVoltage =  new PositionVoltage(target.getRotations());
        sim.setControl(positionVoltage);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(sim.getPosition().getDegrees() - target.getDegrees())<=5;
    }

    @Override
    public void end(boolean interrupted) {
        sim.stop();
    }
}
