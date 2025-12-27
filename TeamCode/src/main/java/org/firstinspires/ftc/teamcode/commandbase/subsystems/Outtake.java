package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.tagVisible;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}

    public static boolean shooting = false;
    public static boolean spinup = false;

    public final MotorEx Tmotor = new MotorEx("Tmotor").reversed();
    public final MotorEx Bmotor = new MotorEx("Bmotor");
    private Servo gateServo;
    public CRServo spinServo1;
    public CRServo spinServo2;
    public Servo led;
    public Servo led2;  // Second LED that mirrors the first

    // Beam break sensor
    private DigitalChannel beamBreak;
    private long beamBrokenStartTime = 0;
    private boolean beamCurrentlyBroken = false;
    private static final long BEAM_BREAK_THRESHOLD_MS = 500;

    // LED flashing state
    private long lastFlashTime = 0;
    private boolean flashState = false;
    private static final long FLASH_INTERVAL_MS = 250;

    // Gradual spindown control
    private double localTargetVel = 0.0;
    private static final double SPINDOWN_RATE = 50.0;

    // Velocity locking for consistent shooting
    private boolean velocityLocked = false;

    // Configurable PID coefficients
    public static double kP = 0.006;
    public static double kI = 0.0002;
    public static double kD = 0.00005;
    public static double kF = 0.00035;

    // Simple PID controllers
    private SimplePID TmotorPID;
    private SimplePID BmotorPID;

    public void initialize(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
        led2 = hardwareMap.get(Servo.class, "led2");
        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Initialize beam break sensor
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        // Initialize PID controllers
        TmotorPID = new SimplePID();
        BmotorPID = new SimplePID();

        reset();
    }

    public void reset() {
        gateServo.setPosition(0.95);
        Tmotor.setPower(0);
        Bmotor.setPower(0);
        spinServo1.setPower(0);
        spinServo2.setPower(0);
        led.setPosition(0.277);
        led2.setPosition(0.277);
        shooting = false;
        spinup = false;
        lastFlashTime = 0;
        flashState = false;
        localTargetVel = 0.0;
        velocityLocked = false;
        beamBrokenStartTime = 0;
        beamCurrentlyBroken = false;

        if (TmotorPID != null) TmotorPID.reset();
        if (BmotorPID != null) BmotorPID.reset();
    }

    @Override
    public void periodic() {
        // Check beam break sensor status
        checkBeamBreak();

        // Update local target velocity with velocity locking
        if (shooting) {
            // Lock velocity at the start of shooting for consistency
            if (!velocityLocked && targetVel > 0.0) {
                localTargetVel = targetVel;
                velocityLocked = true;
            }
            // Don't update localTargetVel again until shooting stops
        } else if (spinup) {
            // Auto-spinup when tag is visible
            velocityLocked = false;
            localTargetVel = targetVel; // Follow the calculated targetVel
        } else {
            // Not shooting and not spinning up = gradual spindown
            velocityLocked = false;
            if (localTargetVel > 0) {
                localTargetVel -= SPINDOWN_RATE;
                if (localTargetVel < 0) {
                    localTargetVel = 0;
                }
            }
        }

        // Apply velocity PID control
        if (localTargetVel > 0) {
            // Get current velocities (RPM)
            double TcurrentVel = Tmotor.getVelocity();
            double BcurrentVel = Bmotor.getVelocity();

            // Calculate PID outputs
            double Tpower = TmotorPID.calculate(TcurrentVel, localTargetVel, shooting);
            double Bpower = BmotorPID.calculate(BcurrentVel, localTargetVel, shooting);

            // Add feed-forward compensation during active feeding
            if (shooting && gateServo.getPosition() < 0.5) { // Gate is open
                Tpower += 0.05; // Compensate for load
                Bpower += 0.05;
            }

            // Clamp to [0, 1]
            Tpower = Math.max(0.0, Math.min(1.0, Tpower));
            Bpower = Math.max(0.0, Math.min(1.0, Bpower));

            // Apply power
            Tmotor.setPower(Tpower);
            Bmotor.setPower(Bpower); //leave off here: if distance < certain amt then multiply by like 1.1
        } else {
            Tmotor.setPower(0);
            Bmotor.setPower(0);
            TmotorPID.reset();
            BmotorPID.reset();
        }

        // LED control based on state
        updateLED();
    }

    /**
     * Checks the beam break sensor and tracks how long it's been broken
     */
    private void checkBeamBreak() {
        // Read sensor - typically LOW (false) when broken, HIGH (true) when intact
        boolean beamIntact = beamBreak.getState();
        boolean beamBroken = !beamIntact;

        long currentTime = System.currentTimeMillis();

        if (beamBroken) {
            if (!beamCurrentlyBroken) {
                // Beam just broke
                beamBrokenStartTime = currentTime;
                beamCurrentlyBroken = true;
            }
            // Beam is still broken (duration checked in updateLED)
        } else {
            // Beam is intact
            beamCurrentlyBroken = false;
            beamBrokenStartTime = 0;
        }
    }

    /**
     * Returns true if the beam has been broken for more than 500ms
     */
    public boolean isBeamBrokenLongEnough() {
        if (!beamCurrentlyBroken) {
            return false;
        }
        long duration = System.currentTimeMillis() - beamBrokenStartTime;
        return duration >= BEAM_BREAK_THRESHOLD_MS;
    }

    /**
     * Updates LED color based on robot state and beam break status
     * Both LEDs always show the same color
     */
    private void updateLED() {
        long currentTime = System.currentTimeMillis();
        double ledPosition = 0.0;

        // PRIORITY 1: Purple if beam broken for >500ms
        if (isBeamBrokenLongEnough()) {
            ledPosition = 0.833; // PURPLE - beam broken for >500ms
        }
        // PRIORITY 2: Green during shooting
        else if (shooting) {
            ledPosition = 0.500; // GREEN - actively shooting
        }
        // PRIORITY 3: Blue when spun up and ready
        else if (spinup && tagVisible) {
            ledPosition = 0.611; // BLUE - spun up and ready
        }
        // PRIORITY 4: Blue when tag visible
        else if (tagVisible) {
            ledPosition = 0.611; // BLUE - tag visible but not spun up yet
        }
        // PRIORITY 5: Flashing red when no tag
        else {
            // FLASHING RED - no tag
            if (currentTime - lastFlashTime > FLASH_INTERVAL_MS) {
                flashState = !flashState;
                lastFlashTime = currentTime;
            }
            ledPosition = flashState ? 0.277 : 0.0;
        }

        // Set both LEDs to the same color
        led.setPosition(ledPosition);
        led2.setPosition(ledPosition);
    }

    // Public methods for telemetry/debugging
    public double getTmotorError() {
        return localTargetVel - Tmotor.getVelocity();
    }

    public double getBmotorError() {
        return localTargetVel - Bmotor.getVelocity();
    }

    public double getLocalTargetVel() {
        return localTargetVel;
    }

    public boolean getBeamBreakState() {
        return beamCurrentlyBroken;
    }

    public long getBeamBrokenDuration() {
        if (!beamCurrentlyBroken) {
            return 0;
        }
        return System.currentTimeMillis() - beamBrokenStartTime;
    }

    public Command open = new Command() {
        private long start = 0;
        @Override public void start() { start = System.currentTimeMillis(); gateServo.setPosition(0.2); }
        @Override public void update() { if (System.currentTimeMillis() - start > 500) gateServo.setPosition(0.19); }
        @Override public boolean isDone() { return true; }
    };

    public Command close = new Command() {
        private long start = 0;
        @Override public void start() { start = System.currentTimeMillis(); gateServo.setPosition(0.93); }
        @Override public void update() { if (System.currentTimeMillis() - start > 500) gateServo.setPosition(0.95); }
        @Override public boolean isDone() { return true; }
    };

    // Simple PID Controller - reads from configurable static variables
    private static class SimplePID {
        private double integral = 0.0;
        private double previousError = 0.0;
        private long lastTime = 0;

        public SimplePID() {
            this.lastTime = System.currentTimeMillis();
        }

        public double calculate(double current, double target, boolean shooting) {
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - lastTime) / 1000.0;
            if (dt > 0.1 || dt <= 0) dt = 0.02;
            lastTime = currentTime;

            double error = target - current;

            // Integral with adaptive anti-windup (higher limit during shooting)
            integral += error * dt;
            double maxIntegral = shooting ? 1000 : 500;
            if (integral > maxIntegral) integral = maxIntegral;
            if (integral < -maxIntegral) integral = -maxIntegral;

            // Derivative
            double derivative = (error - previousError) / dt;
            previousError = error;

            // PIDF output using configurable coefficients
            double output = (kP * error) + (kI * integral) + (kD * derivative) + (kF * target);

            return output;
        }

        public void reset() {
            integral = 0.0;
            previousError = 0.0;
            lastTime = System.currentTimeMillis();
        }
    }
}