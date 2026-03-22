package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** Spins the shooter using the moving-shot distance map. Stops on end. */
public class ShootOnMoveCommand extends Command {
  private final ShooterSubsystem m_shooter;

  public ShootOnMoveCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    m_shooter.setShooterMapLeadCompensated();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocityTarget(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
