package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class MorotSubsys extends GBSubsystem {

    private final SimpleMotorSimulation simpleMotorSimulation;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1);

    private final VoltageOut voltageOut = new VoltageOut(0);

    public MorotSubsys() {
        this.simpleMotorSimulation = new SimpleMotorSimulation(DCMotor.getFalcon500Foc(1), 1, 0.003);
        this.simpleMotorSimulation.applyConfiguration(Confgig.STEER_MOTOR_CONFIG);
    }

    public void setVelocityControl(double velocity) {
        simpleMotorSimulation.setControl(velocityVoltage.withVelocity(velocity));
    }

    public void setPositionControl(Rotation2d angle) {
        simpleMotorSimulation.setControl(positionVoltage.withPosition(angle.getRotations()));
    }

    public void setVoltageControl(double voltage) {
        simpleMotorSimulation.setControl(voltageOut.withOutput(voltage));
    }

    public void stop() {
        simpleMotorSimulation.stop();
    }

    @Override
    public void periodic() {
        super.periodic();
        Logger.recordOutput("Check/PosRot", simpleMotorSimulation.getPositionRevolutions());
        Logger.recordOutput("Check/Current", simpleMotorSimulation.getCurrent());
        Logger.recordOutput("Check/Voltage", simpleMotorSimulation.getVoltage());
        Logger.recordOutput("Check/VelocityRotPerSec", simpleMotorSimulation.getVelocityRevolutionsPerSecond());
    }

}
