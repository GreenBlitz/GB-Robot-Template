package frc.robot.subsystems.swerve.module.extrainputs;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DriveInputsAutoLogged extends DriveInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("UncoupledVelocityAnglesPerSecond", uncoupledVelocityAnglesPerSecond);
        table.put("UncoupledPositions", uncoupledPositions);
        table.put("VelocityMetersPerSecond", velocityMetersPerSecond);
        table.put("PositionsMeters", positionsMeters);
    }

    @Override
    public void fromLog(LogTable table) {
        uncoupledVelocityAnglesPerSecond = table.get("UncoupledVelocityAnglesPerSecond", uncoupledVelocityAnglesPerSecond);
        uncoupledPositions = table.get("UncoupledPositions", uncoupledPositions);
        velocityMetersPerSecond = table.get("VelocityMetersPerSecond", velocityMetersPerSecond);
        positionsMeters = table.get("PositionsMeters", positionsMeters);
    }

    public DriveInputsAutoLogged clone() {
        DriveInputsAutoLogged copy = new DriveInputsAutoLogged();
        copy.uncoupledVelocityAnglesPerSecond = this.uncoupledVelocityAnglesPerSecond;
        copy.uncoupledPositions = this.uncoupledPositions.clone();
        copy.velocityMetersPerSecond = this.velocityMetersPerSecond;
        copy.positionsMeters = this.positionsMeters.clone();
        return copy;
    }
}
