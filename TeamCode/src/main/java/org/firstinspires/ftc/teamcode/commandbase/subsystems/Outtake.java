package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.tagVisible;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}

    public static boolean shooting = false;
    public static boolean spinup = false;  // NEW: Auto-spinup when tag visible

    public final MotorEx Tmotor = new MotorEx("Tmotor").reversed();
    public final MotorEx Bmotor = new MotorEx("Bmotor");
    private Servo gateServo;
    public CRServo spinServo1;
    public CRServo spinServo2;
    public Servo led;

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
        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

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
        shooting = false;
        spinup = false;
        lastFlashTime = 0;
        flashState = false;
        localTargetVel = 0.0;
        velocityLocked = false;

        if (TmotorPID != null) TmotorPID.reset();
        if (BmotorPID != null) BmotorPID.reset();
    }

    @Override
    public void periodic() {
        // Update local target velocity with velocity locking
        if (shooting) {
            // Lock velocity at the start of shooting for consistency
            if (!velocityLocked && targetVel > 0.0) {
                localTargetVel = targetVel;
                velocityLocked = true;
            }
            // Don't update localTargetVel again until shooting stops
        } else if (spinup) {
            // NEW: Auto-spinup when tag is visible
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
            Bmotor.setPower(Bpower);
        } else {
            Tmotor.setPower(0);
            Bmotor.setPower(0);
            TmotorPID.reset();
            BmotorPID.reset();
        }

        // LED control based on state
        long currentTime = System.currentTimeMillis();

        if (shooting) {
            led.setPosition(0.500); // GREEN - actively shooting
        } else if (spinup && tagVisible) {
            led.setPosition(0.611); // BLUE - spun up and ready
        } else if (tagVisible) {
            led.setPosition(0.611); // BLUE - tag visible but not spun up yet
        } else {
            // FLASHING RED - no tag
            if (currentTime - lastFlashTime > FLASH_INTERVAL_MS) {
                flashState = !flashState;
                lastFlashTime = currentTime;
            }
            led.setPosition(flashState ? 0.277 : 0.0);
        }
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