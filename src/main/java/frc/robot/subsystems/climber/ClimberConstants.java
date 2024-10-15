package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public final class ClimberConstants {
  public static final Measure<Distance> SPOOL_DIAMETER = Inches.of(2);

  public static final Measure<Distance> MINIMUM_POSITION = Inches.of(8.0);
  public static final Measure<Distance> STOWED_POSITION = MINIMUM_POSITION;
  public static final Measure<Distance> ARMS_UP_POSITION = Inches.of(32.0);
  public static final Measure<Distance> QUICK_CLIMB_POSITION = Inches.of(22.0);
  public static final Measure<Distance> TRAP_CLIMB_POSITION = Inches.of(2.0);
}
