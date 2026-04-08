package frc.robot.util;

/**
 * PhysicsShooterSolver - standalone projectile-motion flywheel speed calculator.
 *
 * <p>Extracted from UltraShooter (FRC 539, 2026). Derives the required flywheel
 * surface speed from projectile-motion physics rather than an empirical lookup table.
 *
 * <h2>Setup checklist</h2>
 * <ol>
 *   <li>Measure your robot and fill in the geometry constants below.</li>
 *   <li>Tune {@code FLYWHEEL_EFFICIENCY} on the field (start ~0.5, increase until shots land).</li>
 *   <li>Optionally tune the 3-point Lagrange offset anchors for fine correction.</li>
 *   <li>Call {@link #calculateRequiredVelocityFPS(double)} with your odometry distance.</li>
 * </ol>
 *
 * <h2>Physics model (vacuum)</h2>
 * <pre>
 *   v0 = d * sqrt( g / (2 * cos^2(theta) * (d*tan(theta) - h)) )
 * </pre>
 * With drag enabled, a 60-iteration binary search over forward-Euler simulation replaces
 * the closed-form solution.
 */
public class PhysicsShooterSolver {

    // =====================================================================
    // CONSTANTS - MUST BE TUNED FOR YOUR ROBOT
    // =====================================================================

    // -- Geometry (inches) ------------------------------------------------

    /** Height of the hood/barrel exit above the floor (inches). Measure center of exit. */
    private static final double HOOD_HEIGHT_INCHES = 21.5;

    /** Height of the scoring target center above the floor (inches). From game manual. */
    private static final double TARGET_HEIGHT_INCHES = 72.0;

    /** Horizontal offset from robot center to shooter exit (inches).
     *  Added to odometry distance to get true shooter-to-target range. */
    private static final double SHOOTER_OFFSET_INCHES = 3.75;

    /** Fixed launch angle above horizontal (degrees). Match your hood/barrel angle. */
    private static final double LAUNCH_ANGLE_DEGREES = 20.0;

    // -- Flywheel ---------------------------------------------------------

    /** Ball-to-flywheel velocity transfer efficiency (0-1).
     *  Accounts for slip and compression. Start at ~0.5 and tune on the field.
     *  Applied as: ball_exit_speed = flywheel_surface_speed * efficiency. */
    private static final double FLYWHEEL_EFFICIENCY = 0.64;

    // -- Aerodynamics -----------------------------------------------------

    /** Ball diameter (inches). Used for cross-sectional area in drag calculation. */
    private static final double BALL_DIAMETER_INCHES = 6.0;

    /** Ball mass (lbs). Converted to kg internally for SI physics. */
    private static final double BALL_MASS_LBS = 0.595;

    /** Aerodynamic drag constant B = 0.5 * Cd * rho_air * A_cross (kg/m).
     *  For a 6" foam sphere: Cd~0.47, rho=1.225 kg/m^3, A=pi*(0.0762)^2~0.01824 m^2
     *  => B ~ 0.00525 kg/m.  Set to 0.0 to disable drag (use vacuum formula). */
    private static final double DRAG_COEFFICIENT = 0.00525;

    // -- Lagrange fine-tune offsets (optional) -----------------------------
    // Three anchor points define a parabola added as a % on top of the
    // physics speed. Tune independently on the field. Positive = spin faster.

    private static final double CLOSE_ANCHOR_FT      = 3.0;   // distance (ft)
    private static final double MID_ANCHOR_FT        = 12.0;
    private static final double FAR_ANCHOR_FT        = 20.0;

    private static final double CLOSE_OFFSET_PERCENT = 0.0;   // % at close
    private static final double MID_OFFSET_PERCENT   = 0.0;   // % at mid
    private static final double FAR_OFFSET_PERCENT   = 0.0;   // % at far

    // =====================================================================
    // DERIVED CONSTANTS (do not edit)
    // =====================================================================

    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEGREES);
    private static final double SHOOTER_OFFSET_M = SHOOTER_OFFSET_INCHES * 0.0254;
    private static final double HOOD_HEIGHT_M    = HOOD_HEIGHT_INCHES * 0.0254;
    private static final double TARGET_HEIGHT_M  = TARGET_HEIGHT_INCHES * 0.0254;
    private static final double METERS_TO_FEET   = 3.28084;
    private static final double G                = 9.81; // m/s^2

    // Lagrange basis denominators (precomputed, constant for fixed anchors)
    private static final double LAGRANGE_D0 =
            (CLOSE_ANCHOR_FT - MID_ANCHOR_FT) * (CLOSE_ANCHOR_FT - FAR_ANCHOR_FT);
    private static final double LAGRANGE_D1 =
            (MID_ANCHOR_FT - CLOSE_ANCHOR_FT) * (MID_ANCHOR_FT - FAR_ANCHOR_FT);
    private static final double LAGRANGE_D2 =
            (FAR_ANCHOR_FT - CLOSE_ANCHOR_FT) * (FAR_ANCHOR_FT - MID_ANCHOR_FT);

    // =====================================================================
    // PUBLIC API
    // =====================================================================

    /**
     * Calculates the required flywheel surface velocity (ft/s) to score from
     * the given distance (robot center to target, in meters).
     *
     * @param distanceToTargetMeters  Odometry distance from robot center to target (m).
     * @return Required flywheel surface velocity in ft/s, or 0 if shot is impossible.
     */
    public static double calculateRequiredVelocityFPS(double distanceToTargetMeters) {
        return calculateRequiredVelocityFPS(
                distanceToTargetMeters, FLYWHEEL_EFFICIENCY, DRAG_COEFFICIENT, BALL_MASS_LBS);
    }

    /**
     * Full-parameter overload for runtime tuning via NetworkTables / ShooterTuner.
     *
     * @param distanceToTargetMeters  Odometry distance robot center to target (m).
     * @param flywheelEfficiency      Ball-to-flywheel velocity ratio (0-1).
     * @param dragCoefficient         Aerodynamic drag B = 0.5*Cd*rho*A (kg/m). 0 = vacuum.
     * @param ballMassLbs             Ball mass (lbs).
     * @return Required flywheel surface velocity in ft/s, or 0 if impossible.
     */
    public static double calculateRequiredVelocityFPS(
            double distanceToTargetMeters,
            double flywheelEfficiency,
            double dragCoefficient,
            double ballMassLbs) {
        if (flywheelEfficiency <= 0) return 0;
        final double v0 = calcBallExitSpeedMps(distanceToTargetMeters, dragCoefficient, ballMassLbs);
        if (v0 <= 0) return 0;
        return (v0 / flywheelEfficiency) * METERS_TO_FEET;
    }

    /**
     * Calculates the required velocity with the Lagrange fine-tune offset applied.
     * This is what you'd typically call from your shooter subsystem's periodic().
     *
     * @param distanceToTargetMeters  Odometry distance robot center to target (m).
     * @return Tuned flywheel surface velocity in ft/s.
     */
    public static double calculateTunedVelocityFPS(double distanceToTargetMeters) {
        double base   = calculateRequiredVelocityFPS(distanceToTargetMeters);
        double offset = interpolateOffsetFraction(distanceToTargetMeters);
        return base * (1.0 + offset);
    }

    /**
     * Ball time of flight from hood exit to target (seconds).
     * Useful for shoot-on-the-move lead compensation.
     *
     * @param distanceToTargetMeters  Odometry distance robot center to target (m).
     * @return Time of flight in seconds, or 0 if impossible.
     */
    public static double calculateTimeOfFlightSeconds(double distanceToTargetMeters) {
        return calculateTimeOfFlightSeconds(distanceToTargetMeters, DRAG_COEFFICIENT, BALL_MASS_LBS);
    }

    public static double calculateTimeOfFlightSeconds(
            double distanceToTargetMeters, double dragCoefficient, double ballMassLbs) {
        final double d  = distanceToTargetMeters + SHOOTER_OFFSET_M;
        if (d <= 0) return 0;
        final double v0 = calcBallExitSpeedMps(distanceToTargetMeters, dragCoefficient, ballMassLbs);
        if (v0 <= 0) return 0;
        if (dragCoefficient <= 0) {
            return d / (v0 * Math.cos(LAUNCH_ANGLE_RAD));
        } else {
            final double ballMassKg  = ballMassLbs * 0.453592;
            final double dragPerMass = dragCoefficient / Math.max(ballMassKg, 0.001);
            return simulateTimeOfFlight(v0, d, LAUNCH_ANGLE_RAD, dragPerMass);
        }
    }

    // =====================================================================
    // CORE SOLVER
    // =====================================================================

    /**
     * Returns the ball exit speed (m/s) needed to reach the target.
     * No-drag: closed-form analytic formula.
     * With drag: 60-iteration binary search over Euler simulation.
     */
    private static double calcBallExitSpeedMps(
            double distanceToTargetMeters, double dragCoefficient, double ballMassLbs) {
        final double d = distanceToTargetMeters + SHOOTER_OFFSET_M;
        final double h = TARGET_HEIGHT_M - HOOD_HEIGHT_M;
        if (d <= 0) return 0;

        if (dragCoefficient <= 0) {
            // -- Analytic (vacuum) ----------------------------------------
            // v0 = d * sqrt( g / (2 * cos^2(theta) * (d*tan(theta) - h)) )
            final double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
            final double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);
            final double denom    = 2.0 * cosTheta * cosTheta * (d * tanTheta - h);
            if (denom <= 0) return 0; // arc can't reach - target too high or behind
            return d * Math.sqrt(G / denom);
        } else {
            // -- Numerical (drag) -----------------------------------------
            final double ballMassKg  = ballMassLbs * 0.453592;
            final double dragPerMass = dragCoefficient / Math.max(ballMassKg, 0.001);
            return binarySearchV0(d, h, LAUNCH_ANGLE_RAD, dragPerMass);
        }
    }

    // =====================================================================
    // NUMERICAL SIMULATION HELPERS
    // =====================================================================

    /**
     * Binary-searches for exit speed v0 (m/s) such that the simulated
     * trajectory reaches horizontal distance d at vertical height h.
     * 60 bisection steps ~ 18 digits of precision.
     */
    private static double binarySearchV0(
            double d, double h, double angleRad, double dragPerMass) {
        final double DT = 0.005; // 5 ms timestep
        final double LO = 0.5, HI = 40.0;

        // Feasibility: can max speed even reach the target height?
        if (simulateYAtX(HI, d, h, angleRad, dragPerMass, DT) < h) return 0;

        double lo = LO, hi = HI;
        for (int i = 0; i < 60; i++) {
            double mid = (lo + hi) * 0.5;
            if (simulateYAtX(mid, d, h, angleRad, dragPerMass, DT) < h) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return (lo + hi) * 0.5;
    }

    /**
     * Forward-Euler simulation with quadratic drag.
     * Returns the ball's y-height (m, relative to hood exit) when it
     * crosses targetX meters downrange.
     *
     * Drag model per timestep:
     *   speed = sqrt(vx^2 + vy^2)
     *   vx += (-B/m * speed * vx) * dt
     *   vy += (-g - B/m * speed * vy) * dt
     */
    private static double simulateYAtX(
            double v0, double targetX, double h,
            double angleRad, double dragPerMass, double dt) {
        double vx = v0 * Math.cos(angleRad);
        double vy = v0 * Math.sin(angleRad);
        double x = 0, y = 0, prevX = 0, prevY = 0;
        for (int i = 0; i < 5000; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * dt;
            vy += (-G - dragPerMass * speed * vy) * dt;
            prevX = x; prevY = y;
            x += vx * dt;
            y += vy * dt;
            if (x >= targetX) {
                double t = (x - prevX) > 1e-9 ? (targetX - prevX) / (x - prevX) : 0;
                return prevY + t * (y - prevY);
            }
            if (y < -2.0) break;
        }
        return y;
    }

    /** Simulates time of flight (seconds) to reach targetX meters downrange. */
    private static double simulateTimeOfFlight(
            double v0Mps, double targetX, double angleRad, double dragPerMass) {
        final double DT = 0.005;
        double vx = v0Mps * Math.cos(angleRad);
        double vy = v0Mps * Math.sin(angleRad);
        double x = 0, prevX = 0;
        for (int i = 0; i < 5000; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * DT;
            vy += (-G - dragPerMass * speed * vy) * DT;
            prevX = x;
            x += vx * DT;
            if (x >= targetX) {
                double frac = (x - prevX) > 1e-9 ? (targetX - prevX) / (x - prevX) : 0.0;
                return (i + frac) * DT;
            }
        }
        return 0;
    }

    // =====================================================================
    // LAGRANGE FINE-TUNE OFFSET
    // =====================================================================

    /**
     * 3-point Lagrange quadratic interpolation of the fine-tune speed offset.
     * Returns a fraction (e.g. 5% = 0.05). Distance clamped to anchor range.
     */
    private static double interpolateOffsetFraction(double distanceMeters) {
        final double d = Math.max(CLOSE_ANCHOR_FT,
                Math.min(FAR_ANCHOR_FT, distanceMeters * METERS_TO_FEET));
        final double l0 = (d - MID_ANCHOR_FT) * (d - FAR_ANCHOR_FT) / LAGRANGE_D0;
        final double l1 = (d - CLOSE_ANCHOR_FT) * (d - FAR_ANCHOR_FT) / LAGRANGE_D1;
        final double l2 = (d - CLOSE_ANCHOR_FT) * (d - MID_ANCHOR_FT) / LAGRANGE_D2;
        return (CLOSE_OFFSET_PERCENT * l0 + MID_OFFSET_PERCENT * l1 + FAR_OFFSET_PERCENT * l2) / 100.0;
    }
}
