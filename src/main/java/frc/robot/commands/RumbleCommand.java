package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Rumbles the controller at a given intensity for a fixed duration, then stops.
 * Does not require any subsystem — safe to run in parallel with anything.
 */
public class RumbleCommand extends Command {
  private final CommandXboxController m_controller;
  private final double m_intensity;
  private final double m_durationSec;
  private final Timer m_timer = new Timer();

  public RumbleCommand(CommandXboxController controller, double intensity, double durationSec) {
    m_controller = controller;
    m_intensity = intensity;
    m_durationSec = durationSec;
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, m_intensity);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_durationSec);
  }

  @Override
  public void end(boolean interrupted) {
    m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    m_timer.stop();
  }
}
