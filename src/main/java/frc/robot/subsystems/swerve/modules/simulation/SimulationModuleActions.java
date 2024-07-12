package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;

class SimulationModuleActions {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    private final PositionVoltage steerPositionRequest;
    private final VoltageOut steerVoltageRequest;
    private final VoltageOut driveVoltageRequest;

    protected SimulationModuleActions(SimulationModuleConstants constants) {
        this.steerMotor = constants.getSteerMotor();
        this.driveMotor = constants.getDriveMotor();

        this.steerPositionRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFocSteer());
        this.steerVoltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFocSteer());
        this.driveVoltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFocDrive());
    }

    protected void stop() {
        steerMotor.stop();
        driveMotor.stop();
    }

    protected void setTargetSteerVoltage(double voltage) {
        steerMotor.setControl(steerVoltageRequest.withOutput(voltage));
    }
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    protected void setTargetDriveVoltage(double voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

}
