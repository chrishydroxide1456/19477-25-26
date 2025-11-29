package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.controllable.RunToVelocity;
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
    private Servo led;

    private final ControlSystem outcontroller = ControlSystem.builder()
            .velPid(0.01, 0.0, 0.0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    // Initialize servos
    public void initialize(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(1.0);
    }

    // ---------------------------------------------------------
    // Flywheel background control
    // ---------------------------------------------------------
    public Command on = new Command() {
        @Override
        public void update() {
            if (Tmotor.getVelocity() > targetVel - 40) {
                led.setPosition(0.5);
            } else {
                led.setPosition(0.3);
            }
        }
        @Override public boolean isDone() { return false; }
    };

    public Command off = new Command() {
        long t1 = System.currentTimeMillis();
        @Override
        public void update() {
            long t2 = System.currentTimeMillis();
            double dt = (t2 - t1) / 1000.0;
            t1 = t2;

            if (Tmotor.getVelocity() > 0) {
                targetVel = Math.max(0, Tmotor.getVelocity() - 400 * dt);
            }
        }
        @Override public boolean isDone() { return Tmotor.getVelocity() == 0.0; }
        @Override public void stop(boolean interrupted) { targetVel = 0.0; }
    };

    // ---------------------------------------------------------
    // Gate control
    // ---------------------------------------------------------
    public Command open = new Command() {
        private long startTime;
        @Override
        public void start() {
            startTime = System.currentTimeMillis();
            spinServo1.setPower(1.0);
            spinServo2.setPower(1.0);
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
            spinServo1.setPower(0.0);
            spinServo2.setPower(0.0);
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

    // ---------------------------------------------------------
    // Flywheel velocity commands (for sequences)
    // ---------------------------------------------------------
    public Command reverse = new RunToVelocity(outcontroller, -56);
    public Command teston2000 = new RunToVelocity(outcontroller, 933);
    public Command teston4000 = new RunToVelocity(outcontroller, 1866);
    public Command testoff = new RunToVelocity(outcontroller, 0.0);

    // ---------------------------------------------------------
    // PID Loop - must be called from OpMode
    // ---------------------------------------------------------
    public void periodic() {
        outcontroller.setGoal(new KineticState(0.0, targetVel, 0.0));
        Tmotor.setPower(outcontroller.calculate(Tmotor.getState()));
        Bmotor.setPower(outcontroller.calculate(Bmotor.getState()));
    }
}
