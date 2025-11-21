# FTC Robot Setup Guide - NextFTC

## What Was Done

Your FTC robot code has been restructured to use the **NextFTC framework** following best practices from competitive teams. The code is now clean, simple, and ready to test on your robot.

## Robot Configuration

Your robot is configured for:
- **4-wheel mecanum drive**
- **1 intake motor**
- **2 outtake motors** (working together)
- **1 servo blocker** (to control when elements are released)

## Hardware Configuration File

Make sure your robot configuration in the Driver Station matches these names:

### Drive Motors
- `FLmotor` - Front Left Motor
- `FRmotor` - Front Right Motor
- `BLmotor` - Back Left Motor
- `BRmotor` - Back Right Motor

### Intake
- `inmotor` - Intake Motor

### Outtake
- `Tmotor` - Top Outtake Motor
- `Bmotor` - Bottom Outtake Motor
- `gateServo` - Servo Blocker

## How to Use

### 1. On the Driver Station
1. Select "TeleOp" from the OpMode list
2. Initialize the robot
3. Start when ready

### 2. Control Your Robot
See [CONTROLS.md](CONTROLS.md) for the complete control map.

**Quick controls:**
- **Drive**: Use left stick to move, right stick to turn
- **Intake**: Right trigger to run intake
- **Outtake**: Right bumper to run outtake motors
- **Blocker**: D-pad Up to open, D-pad Down to close

## Code Structure

```
TeamCode/
├── commandbase/
│   ├── subsystems/
│   │   ├── Drive.java       # Mecanum drive control
│   │   ├── Intake.java      # Single intake motor
│   │   └── Outtake.java     # Two motors + servo
│   ├── Routines.java        # Common command sequences
│   └── AutoRoutines.java    # Autonomous routines
└── opmodes/
    ├── TeleOp.java          # Main driver control
    └── Auto.java            # Autonomous mode
```

## Key Features

### NextFTC Framework
- **Command-based architecture**: Actions are organized as commands
- **Subsystem pattern**: Each mechanism (drive, intake, outtake) is a subsystem
- **Automatic scheduling**: Commands run automatically when triggered

### Clean Code
- Simple, readable structure
- Easy to modify and tune
- Well-documented controls
- Ready for competition

## Testing Checklist

Before your first run:
- [ ] Verify hardware configuration names match
- [ ] Test drive motors individually
- [ ] Test intake motor direction
- [ ] Test outtake motors direction
- [ ] Test servo blocker positions (0.0 = closed, 1.0 = open)
- [ ] Verify motor directions (use `.reversed()` if needed)

## Tuning

If you need to adjust:

### Motor Speeds
Edit the power values in subsystem files:
- **Intake.java**: Lines 23-27 (currently 1.0, 0.4, -0.2, etc.)
- **Outtake.java**: Lines 34-47 (currently 1.0 for both motors)

### Servo Positions
Edit **Outtake.java**:
- Line 30: Initial position (currently 0.0 = closed)
- Line 50: Open position (currently 1.0)
- Line 51: Close position (currently 0.0)

### Drive Sensitivity
Edit **Drive.java**:
- Line 24: Strafe correction (currently 1.1)

## Next Steps

1. **Test on Robot**: Deploy and test all controls
2. **Tune Values**: Adjust motor powers and servo positions as needed
3. **Add Autonomous**: Use AutoRoutines.java to create autonomous sequences
4. **Practice**: Get comfortable with the controls before competition

## Need Help?

- Check [CONTROLS.md](CONTROLS.md) for button mappings
- Review code comments in each subsystem file
- Test each mechanism individually before running full sequences

## Competition Ready

Your code follows FTC best practices and is structured like competitive teams' code. The simple design makes it easy to:
- Understand what's happening
- Fix problems quickly
- Make adjustments during competition
- Add new features as needed

Good luck at competition! 🤖🏆
