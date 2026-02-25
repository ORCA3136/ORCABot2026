package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelPathConstants;
import frc.robot.RobotLogger;
import frc.robot.subsystems.KickerSubsystem;

/**
 * Runs the kicker at a given speed with jam protection.
 *
 * State machine: RUNNING -> REVERSING -> (retry or FAILED)
 * - RUNNING: Kicker at forward speed. If current exceeds threshold for detection time, reverse.
 * - REVERSING: Kicker at reverse speed for a set duration, then retry.
 * - FAILED: After max retries, stop kicker and end the command.
 */
public class KickerJamProtectionCommand extends Command {

  private enum State { RUNNING, REVERSING, FAILED }

  private final KickerSubsystem m_kicker;
  private final double m_forwardSpeed;

  private State m_state;
  private int m_retryCount;
  private final Timer m_timer = new Timer();

  public KickerJamProtectionCommand(KickerSubsystem kicker, double forwardSpeed) {
    m_kicker = kicker;
    m_forwardSpeed = forwardSpeed;
    addRequirements(kicker);
  }

  @Override
  public void initialize() {
    m_state = State.RUNNING;
    m_retryCount = 0;
    m_timer.restart();
    m_kicker.setKickerVelocity(m_forwardSpeed);
  }

  @Override
  public void execute() {
    switch (m_state) {
      case RUNNING:
        double current = m_kicker.getKickerCurrent();
        if (current > FuelPathConstants.kJamCurrentThresholdAmps) {
          if (m_timer.hasElapsed(FuelPathConstants.kJamDetectionTimeSec)) {
            // Jam detected — reverse
            m_retryCount++;
            RobotLogger.log("FUEL JAM detected (attempt " + m_retryCount + "/" + FuelPathConstants.kJamMaxRetries
                + ", current: " + String.format("%.1f", current) + "A)");
            if (m_retryCount > FuelPathConstants.kJamMaxRetries) {
              m_state = State.FAILED;
              m_kicker.setKickerVelocity(0);
              RobotLogger.log("FUEL JAM FAILED — max retries exceeded");
              DriverStation.reportWarning("Kicker jam protection: max retries exceeded", false);
            } else {
              m_state = State.REVERSING;
              m_timer.restart();
              m_kicker.setKickerVelocity(FuelPathConstants.kJamReverseSpeed);
            }
          }
        } else {
          // Current is below threshold — reset the detection timer
          m_timer.restart();
        }
        break;

      case REVERSING:
        if (m_timer.hasElapsed(FuelPathConstants.kJamReverseDurationSec)) {
          // Done reversing — try forward again
          m_state = State.RUNNING;
          m_timer.restart();
          m_kicker.setKickerVelocity(m_forwardSpeed);
          RobotLogger.log("FUEL JAM retrying forward (attempt " + m_retryCount + "/" + FuelPathConstants.kJamMaxRetries + ")");
        }
        break;

      case FAILED:
        // Command will end via isFinished()
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_kicker.setKickerVelocity(0);
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_state == State.FAILED;
  }
}
