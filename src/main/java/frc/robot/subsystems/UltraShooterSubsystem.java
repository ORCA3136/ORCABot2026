// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NetworkTableNames;
import frc.robot.util.PhysicsShooterSolver;

/**
 * Virtual subsystem that runs the physics-based shooter solver in parallel
 * with the existing lookup-table ShooterSubsystem. No motors — only
 * calculates speeds and publishes to NetworkTables for dashboard comparison.
 */
public class UltraShooterSubsystem extends SubsystemBase {

  private final SwerveSubsystem m_swerveSubsystem;

  // -- NetworkTables telemetry --
  private final NetworkTable ultraTable =
      NetworkTableInstance.getDefault().getTable(NetworkTableNames.UltraShooter.kTable);

  private final NetworkTableEntry physicsRPMEntry   = ultraTable.getEntry(NetworkTableNames.UltraShooter.kPhysicsRPM);
  private final NetworkTableEntry tunedRPMEntry     = ultraTable.getEntry(NetworkTableNames.UltraShooter.kTunedRPM);
  private final NetworkTableEntry tofEntry          = ultraTable.getEntry(NetworkTableNames.UltraShooter.kTimeOfFlightSec);
  private final NetworkTableEntry distanceEntry     = ultraTable.getEntry(NetworkTableNames.UltraShooter.kDistanceM);
  private final NetworkTableEntry deltaRPMEntry     = ultraTable.getEntry(NetworkTableNames.UltraShooter.kDeltaRPM);

  // -- Cached results for public getters --
  private double lastPhysicsRPM = 0;
  private double lastTunedRPM = 0;
  private double lastTimeOfFlight = 0;

  // -- Flywheel geometry for ft/s -> RPM conversion --
  // RPM = fps / (PI * diameter_ft / 60)  =  fps * 60 / (PI * diameter_ft)
  // TODO: MEASURE your flywheel diameter and update this value
  private static final double FLYWHEEL_DIAMETER_INCHES = 4.0;
  private static final double FLYWHEEL_CIRCUMFERENCE_FT = (Math.PI * FLYWHEEL_DIAMETER_INCHES) / 12.0;

  public UltraShooterSubsystem(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {
    double distanceM = m_swerveSubsystem.getDistanceToHubMeters();

    // Raw physics speed (no Lagrange offset)
    double physicsFPS = PhysicsShooterSolver.calculateRequiredVelocityFPS(distanceM);
    lastPhysicsRPM = fpsToRPM(physicsFPS);

    // Tuned physics speed (with Lagrange offset)
    double tunedFPS = PhysicsShooterSolver.calculateTunedVelocityFPS(distanceM);
    lastTunedRPM = fpsToRPM(tunedFPS);

    // Time of flight for lead compensation
    lastTimeOfFlight = PhysicsShooterSolver.calculateTimeOfFlightSeconds(distanceM);

    // Read the existing shooter's target for delta comparison
    double lookupRPM = NetworkTableInstance.getDefault()
        .getTable(NetworkTableNames.Shooter.kTable)
        .getEntry(NetworkTableNames.Shooter.kTargetRPM)
        .getDouble(0);

    // Publish to NetworkTables
    physicsRPMEntry.setDouble(lastPhysicsRPM);
    tunedRPMEntry.setDouble(lastTunedRPM);
    tofEntry.setDouble(lastTimeOfFlight);
    distanceEntry.setDouble(distanceM);
    deltaRPMEntry.setDouble(lastTunedRPM - lookupRPM);
  }

  /** Returns the last-computed physics-based target RPM (with Lagrange offset). */
  public double getPhysicsTargetRPM() {
    return lastTunedRPM;
  }

  /** Returns the last-computed time of flight in seconds. */
  public double getTimeOfFlightSeconds() {
    return lastTimeOfFlight;
  }

  /** Converts flywheel surface speed (ft/s) to flywheel RPM. */
  private static double fpsToRPM(double fps) {
    if (FLYWHEEL_CIRCUMFERENCE_FT <= 0) return 0;
    return (fps / FLYWHEEL_CIRCUMFERENCE_FT) * 60.0;
  }
}
