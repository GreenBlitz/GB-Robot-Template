package frc.robot.subsystems.swerve.simulationswerve.simulationmodule;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;

public class SimulationModuleActions {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    private final PositionVoltage steerPositionRequest = new PositionVoltage(0);

    private final VoltageOut driveVoltageRequest = new VoltageOut(0);

    public SimulationModuleActions(SimulationModuleConfigObject simulationModuleConfigObject) {
        this.driveMotor = simulationModuleConfigObject.getDriveMotor();
        this.steerMotor = simulationModuleConfigObject.getSteerMotor();
    }

    public void setTargetOpenLoopVelocity(double voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    public void stop() {
        driveMotor.stop();
        steerMotor.stop();
    }

}
