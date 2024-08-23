package frc.robot.subsystems.swerve.modules.steer.talonfx;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.talonfx.TalonFXMotor;
import frc.robot.hardware.talonfx.TalonFXSignals;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerThreadInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.Queue;

public class TalonFXSteer extends TalonFXMotor implements ISteer {

    private final Queue<Double> positionQueue;

    public TalonFXSteer(TalonFXWrapper motor, TalonFXSignals signals, SysIdRoutine.Config config) {
        super(motor, signals, config);
        this.positionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(motor, signals.position(), signals.velocity());
    }

    @Override
    public void updateInputs(SteerThreadInputsAutoLogged inputs) {
        inputs.angleOdometrySamples = positionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        positionQueue.clear();
    }

}
