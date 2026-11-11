package frc.robot.vision.cameras.limelight.inputs;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MTInputsAutoLogged extends MTInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MtRawData", mtRawData);
    table.put("PrimaryTagPoseInCameraSpace", primaryTagPoseInCameraSpace);
  }

  @Override
  public void fromLog(LogTable table) {
    mtRawData = table.get("MtRawData", mtRawData);
    primaryTagPoseInCameraSpace =
        table.get("PrimaryTagPoseInCameraSpace", primaryTagPoseInCameraSpace);
  }

  public MTInputsAutoLogged clone() {
    MTInputsAutoLogged copy = new MTInputsAutoLogged();
    copy.mtRawData = this.mtRawData;
    copy.primaryTagPoseInCameraSpace = this.primaryTagPoseInCameraSpace;
    return copy;
  }
}
