# TeleOp Control Map

## Robot Configuration
- **Drive System**: 4-wheel mecanum drive (FLmotor, FRmotor, BLmotor, BRmotor)
- **Intake**: Single motor (inmotor)
- **Outtake**: Two motors (Tmotor, Bmotor) + servo blocker (gateServo)

## Gamepad 1 Controls

### Drive
- **Left Stick**: Forward/backward and strafe left/right
- **Right Stick X**: Rotate robot left/right

### Intake
- **Right Trigger**: Run intake forward (hold to run)
- **Left Trigger**: Reverse intake (hold to reverse)
- **A Button**: Start intake
- **B Button**: Stop intake
- **X Button**: Reverse intake

### Outtake
- **Right Bumper**: Run outtake motors (hold to run)
- **Left Bumper**: Reverse outtake motors (hold to reverse)
- **Y Button**: Stop outtake motors
- **D-pad Up**: Open servo blocker
- **D-pad Down**: Close servo blocker

## Quick Reference

### Score Sequence (Manual)
1. Hold **Right Bumper** to spin up outtake motors
2. Press **D-pad Up** to open servo blocker
3. Release game element
4. Press **D-pad Down** to close servo blocker
5. Release **Right Bumper** to stop motors

### Intake Sequence (Manual)
1. Hold **Right Trigger** to run intake
2. Release when element is secured
3. Use **Left Trigger** if you need to reverse/eject

## Hardware Configuration Names
Make sure these match your robot's configuration file:
- Drive motors: `FLmotor`, `FRmotor`, `BLmotor`, `BRmotor`
- Intake motor: `inmotor`
- Outtake motors: `Tmotor`, `Bmotor`
- Gate servo: `gateServo`
