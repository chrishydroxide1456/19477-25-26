package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.tagVisible;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}

    public static boolean shooting = false;

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

    // Simple PID controllers
    private SimplePID TmotorPID;
    private SimplePID BmotorPID;

    public void initialize(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Initialize PID - tune these values as needed
        TmotorPID = new SimplePID(0.0004, 0.00008, 0.00002, 0.00035);
        BmotorPID = new SimplePID(0.0004, 0.00008, 0.00002, 0.00035);

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
        lastFlashTime = 0;
        flashState = false;
        localTargetVel = 0.0;

        if (TmotorPID != null) TmotorPID.reset();
        if (BmotorPID != null) BmotorPID.reset();
    }

    @Override
    public void periodic() {
        // Update local target velocity for gradual spindown
        if (shooting && targetVel > 0.0) {
            localTargetVel = targetVel;
        } else if (localTargetVel > 0) {
            localTargetVel -= SPINDOWN_RATE;
            if (localTargetVel < 0) {
                localTargetVel = 0;
            }
        }

        // Apply velocity PID control
        if (localTargetVel > 0) {
            // Get current velocities (RPM)
            double TcurrentVel = Tmotor.getVelocity();
            double BcurrentVel = Bmotor.getVelocity();

            // Calculate PID outputs
            double Tpower = TmotorPID.calculate(TcurrentVel, localTargetVel);
            double Bpower = BmotorPID.calculate(BcurrentVel, localTargetVel);

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
            led.setPosition(0.500); // GREEN
        } else if (tagVisible) {
            led.setPosition(0.611); // BLUE
        } else {
            // FLASHING RED
            if (currentTime - lastFlashTime > FLASH_INTERVAL_MS) {
                flashState = !flashState;
                lastFlashTime = currentTime;
            }
            led.setPosition(flashState ? 0.277 : 0.0);
        }
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

    // Simple PID Controller
    private static class SimplePID {
        private final double kP, kI, kD, kF;
        private double integral = 0.0;
        private double previousError = 0.0;
        private long lastTime = 0;

        public SimplePID(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.lastTime = System.currentTimeMillis();
        }

        public double calculate(double current, double target) {
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - lastTime) / 1000.0;
            if (dt > 0.1 || dt <= 0) dt = 0.02;
            lastTime = currentTime;

            double error = target - current;

            // Integral with anti-windup
            integral += error * dt;
            if (integral > 500) integral = 500;
            if (integral < -500) integral = -500;

            // Derivative
            double derivative = (error - previousError) / dt;
            previousError = error;

            // PIDF output
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