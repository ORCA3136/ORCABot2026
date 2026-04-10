package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

/**
 * Plays a rumble pattern on the controller defined by alternating
 * (intensity, duration) steps, then stops.
 * Does not require any subsystem — safe to run in parallel with anything.
 */
public class RumbleCommand extends Command {
  private final CommandXboxController m_controller;
  private final double[] m_intensities;
  private final double[] m_durations;
  private final Timer m_timer = new Timer();
  private int m_stepIndex;
  private double m_stepEndTime;

  /**
   * @param controller the Xbox controller to rumble
   * @param pattern    alternating intensity, duration pairs:
   *                   [intensity1, duration1, intensity2, duration2, ...]
   */
  public RumbleCommand(CommandXboxController controller, double... pattern) {
    m_controller = controller;
    int steps = pattern.length / 2;
    m_intensities = new double[steps];
    m_durations = new double[steps];
    for (int i = 0; i < steps; i++) {
      m_intensities[i] = pattern[i * 2];
      m_durations[i] = pattern[i * 2 + 1];
    }
  }

  /** Medium double-pulse when intake roller starts. */
  public static Command doubleRumble(CommandXboxController controller) {
    return new RumbleCommand(controller,
        OperatorConstants.kDoubleRumbleIntensity, OperatorConstants.kDoubleRumbleOnSec,
        0.0, OperatorConstants.kDoubleRumbleGapSec,
        OperatorConstants.kDoubleRumbleIntensity, OperatorConstants.kDoubleRumbleOnSec);
  }

  /** Intense rapid pulsing when intake auto-cuts off (no fuel timeout). */
  public static Command earthquake(CommandXboxController controller) {
    double onSec = OperatorConstants.kEarthquakeOnSec;
    double offSec = OperatorConstants.kEarthquakeOffSec;
    double intensity = OperatorConstants.kEarthquakeIntensity;
    double totalSec = OperatorConstants.kEarthquakeTotalSec;

    // Build repeating on/off pairs to fill the total duration
    int cycles = (int) Math.ceil(totalSec / (onSec + offSec));
    double[] pattern = new double[cycles * 4]; // 2 steps per cycle, 2 values per step
    for (int i = 0; i < cycles; i++) {
      pattern[i * 4] = intensity;
      pattern[i * 4 + 1] = onSec;
      pattern[i * 4 + 2] = 0.0;
      pattern[i * 4 + 3] = offSec;
    }
    return new RumbleCommand(controller, pattern);
  }

  @Override
  public void initialize() {
    m_stepIndex = 0;
    m_timer.restart();
    m_stepEndTime = m_durations[0];
    m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, m_intensities[0]);
  }

  @Override
  public void execute() {
    double elapsed = m_timer.get();
    while (m_stepIndex < m_intensities.length && elapsed >= m_stepEndTime) {
      m_stepIndex++;
      if (m_stepIndex < m_intensities.length) {
        m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, m_intensities[m_stepIndex]);
        m_stepEndTime += m_durations[m_stepIndex];
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_stepIndex >= m_intensities.length;
  }

  @Override
  public void end(boolean interrupted) {
    m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    m_timer.stop();
  }
}
