# Button Bindings — Xbox Controller (Port 0)

Toggle between modes by setting `useProductionBindings` in `RobotContainer.java:67`.

---

## Production Bindings (`configureBindings`)

### Face Buttons
| Button | Action | Type |
|--------|--------|------|
| A | Stop shooter (set target 0 RPM) | onTrue |
| B | Increase shooter velocity (+5 RPM) | onTrue |
| X | Decrease shooter velocity (-100 RPM) | onTrue |
| Y | Set shooter to 500 RPM | onTrue |

### Center Buttons
| Button | Action | Type |
|--------|--------|------|
| Start | Zero gyro | onTrue |
| Back | *(unbound)* | — |

### Sticks (press)
| Button | Action | Type |
|--------|--------|------|
| Left Stick | Climber extend (1000 RPM-scale) | whileTrue |
| Right Stick | Climber retract (-1000 RPM-scale) | whileTrue |

### D-Pad
| Button | Action | Type |
|--------|--------|------|
| D-Pad Up | Conveyor + kicker forward (500 / 1500) | whileTrue |
| D-Pad Down | Conveyor + kicker reverse (-1000 / -1000) | whileTrue |
| D-Pad Left | Intake roller in (6000) | whileTrue |
| D-Pad Right | Slow hood oscillate (testing) | whileTrue |

### Bumpers
| Button | Action | Type |
|--------|--------|------|
| Right Bumper | Hood angle +1 degree | onTrue |
| Left Bumper | Hood angle -1 degree | onTrue |

### Triggers
| Button | Action | Type |
|--------|--------|------|
| Left Trigger (>30%) | Intake deploy forward (3000) / stop on release | onTrue/onFalse |
| Right Trigger (>30%) | Intake deploy reverse (-3000) / stop on release | onTrue/onFalse |

### Joysticks (analog)
| Stick | Action |
|-------|--------|
| Left Stick | Drive translation (field-oriented) |
| Right Stick X | Drive rotation |

---

## Test Bindings (`configureTestBindings`)

### Face Buttons
| Button | Action | Type |
|--------|--------|------|
| A | Increase shooter velocity level 1 (+5 RPM) | onTrue |
| B | Increase shooter velocity level 2 (+50 RPM) | onTrue |
| X | Increase shooter velocity level 3 (+100 RPM) | onTrue |
| Y | Increase shooter velocity level 4 (+500 RPM) | onTrue |

### Center Buttons
| Button | Action | Type |
|--------|--------|------|
| Start | Zero gyro | onTrue |
| Back | *(unbound)* | — |

### Sticks (press)
| Button | Action | Type |
|--------|--------|------|
| Left Stick | Set hood target to 0 degrees | onTrue |
| Right Stick | Set hood target to 30 degrees | onTrue |

### D-Pad
| Button | Action | Type |
|--------|--------|------|
| D-Pad Up | Conveyor + kicker forward (500 / 1500) | whileTrue |
| D-Pad Down | Conveyor + kicker reverse (-1000 / -1000) | whileTrue |
| D-Pad Left | Intake roller in (4000) | whileTrue |
| D-Pad Right | Intake roller stop (0) | whileTrue |

### Bumpers
| Button | Action | Type |
|--------|--------|------|
| Right Bumper | Hood angle +1 degree | onTrue |
| Left Bumper | Hood angle -1 degree | onTrue |

### Triggers
| Button | Action | Type |
|--------|--------|------|
| Left Trigger (>30%) | Full fuel path (intake deploy + oscillate + roller + conveyor + kicker) | whileTrue |
| Right Trigger (>30%) | Toggle shooter increment direction (hold = reverse) | onTrue/onFalse |

### Joysticks (analog)
| Stick | Action |
|-------|--------|
| Left Stick | Drive translation (field-oriented) |
| Right Stick X | Drive rotation |

---

## Named Commands (PathPlanner Autonomous)

| Name | Action |
|------|--------|
| Run Intake | Intake roller at 6000 (holds until interrupted) |
| Stop Intake | Intake roller to 0 (instant) |
| Deploy Intake | Set intake deployed (instant) |
| Retract Intake | Set intake retracted (instant) |
| Stop Shooter | Shooter target to 0 RPM (instant) |
| Run Shooter Low | Shooter target to 1900 RPM (instant) |
| Run Shooter Medium | Shooter target to 3500 RPM (instant) |
| Run Shooter High | Shooter target to 5000 RPM (instant) |
| Hood Position Low | Hood to 1 degree (instant) |
| Hood Position Medium | Hood to 10 degrees (instant) |
| Hood Position High | Hood to 20 degrees (instant) |
| Run Conveyor | Conveyor 500 + kicker 4000 (holds until interrupted) |
| Stop Conveyor | Conveyor + kicker to 0 (instant) |
| Full Fuel Path | Deploy + oscillate + intake + conveyor + kicker (holds) |
| Stop Fuel Path | Stop all fuel motors + retract intake (instant) |
| Intake And Conveyor | Intake roller + conveyor (holds) |
| Metered Feed | Conveyor always + kicker only when shooter ready (holds) |
| Kicker Pulse | Single kicker burst 150ms (runs once) |
| Emergency Reverse | All fuel motors reverse at high speed (holds) |

---

## Binding Types Quick Reference

| Type | Behavior |
|------|----------|
| **onTrue** | Runs once when button is first pressed |
| **onFalse** | Runs once when button is released |
| **whileTrue** | Runs while held, stops on release |
