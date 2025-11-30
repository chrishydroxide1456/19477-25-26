package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}

    public final MotorEx Tmotor = new MotorEx("Tmotor").reversed();
    public final MotorEx Bmotor = new MotorEx("Bmotor");
    private Servo gateServo;
    private CRServo spinServo1;
    private CRServo spinServo2;
    public Servo led;

    public void initialize(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(1.0);
    }

    // ---------------- SIMPLE LED / FLYWHEEL STATE ----------------

    // Called when Limelight says "we have a target and want to shoot"
    public void periodic() {
        // For now: fixed power when targetVel > 0, else off.
        // Once this works, swap back to PID velocity control.
        if (targetVel > 0.0) {
            Tmotor.setPower(0.7);
            Bmotor.setPower(0.7);
            led.setPosition(0.5); // "ready" color
        } else {
            Tmotor.setPower(0.0);
            Bmotor.setPower(0.0);
            led.setPosition(0.3); // "idle" color
        }
    }

    // ---------------- INTAKE GATE / SPINNERS ----------------

    public Command spinReverse = new Command() {
        @Override
        public void update() {
            spinServo1.setPower(-1.0);
            spinServo2.setPower(-1.0);
        }
        @Override public boolean isDone() { return false; }
    };

    public Command stopSpin = new Command() {
        @Override
        public void update() {
            spinServo1.setPower(0.0);
            spinServo2.setPower(0.0);
        }
        @Override public boolean isDone() { return false; }
    };

    public Command open = new Command() {
        private long startTime;
        @Override
        public void start() {
            startTime = System.currentTimeMillis();
            gateServo.setPosition(0.2);
        }
        @Override
        public void update() {
            if (System.currentTimeMillis() - startTime > 1000) {
                gateServo.setPosition(0.19);
            }
        }
        @Override public boolean isDone() { return gateServo.getPosition() == 0.19; }
    };

    public Command close = new Command() {
        private long startTime1;
        @Override
        public void start() {
            startTime1 = System.currentTimeMillis();
            gateServo.setPosition(0.97);
        }
        @Override
        public void update() {
            if (System.currentTimeMillis() - startTime1 > 1000) {
                gateServo.setPosition(1.0);
            }
        }
        @Override public boolean isDone() { return gateServo.getPosition() == 1.0; }
    };

    // You can re-add PID-based velocity commands later once basic spin works
}
