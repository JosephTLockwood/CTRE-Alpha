package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.RobotBase;

public final class RobotMode {

  public static Mode getMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    } else if (RobotBase.isSimulation() && Utils.isReplay()) {
      return Mode.REPLAY;
    } else {
      return Mode.SIM;
    }
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
