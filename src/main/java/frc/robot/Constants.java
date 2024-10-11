package frc.robot;

public final class Constants {

  public static Mode getMode() {
    return Mode.SIM;
    // return Mode.REPLAY;
    // if (RobotBase.isReal()) {
    //   return Mode.REAL;
    // } else if (RobotBase.isSimulation() && HootReplay.isPlaying()) {
    //   return Mode.REPLAY;
    // } else {
    //   return Mode.SIM;
    // }
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
