package frc.robot.subsystems.swerve.modules.steer.simulation;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.steer.ISteer;

public class SimulationSteer implements ISteer {

    private final SimpleMotorSimulation steerMotor;

    private final PositionVoltage positionRequest;
    private final VoltageOut voltageRequest;

    public SimulationSteer(SimulationSteerConstants constants){
        this.steerMotor = constants.getMotor();

        this.positionRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFOC());
        this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());
    }

    @Override
    public void setBrake(boolean brake) {

    }

    @Override
    public void resetToAngle(Rotation2d angle) {

    }


    @Override
    public void stop() {
        steerMotor.stop();
    }

    @Override
    public void setVoltage(double voltage) {
        steerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(positionRequest.withPosition(angle.getRotations()));
    }


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        inputs.getSteerMotorInputs().angle = steerMotor.getPosition();
        inputs.getSteerMotorInputs().velocity = steerMotor.getVelocity();
        inputs.getSteerMotorInputs().voltage = steerMotor.getVoltage();
        inputs.getSteerMotorInputs().angleOdometrySamples = new Rotation2d[]{inputs.getSteerMotorInputs().angle};
    }

}
