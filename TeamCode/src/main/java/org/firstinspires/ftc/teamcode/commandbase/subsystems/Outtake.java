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
    public CRServo spinServo1;  // CHANGED: private → public
    public CRServo spinServo2;  // CHANGED: private → public
    public Servo led;

    public void initialize(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        reset();
    }

    public void reset() {
        gateServo.setPosition(1.0);
        Tmotor.setPower(0);
        Bmotor.setPower(0);
        spinServo1.setPower(0);
        spinServo2.setPower(0);
        led.setPosition(0.277);
        shooting = false;
    }

    @Override
    public void periodic() {
        if (shooting && targetVel > 0.0) {
            double power = 0.25 + (targetVel / 3000.0);
            power = Math.max(0.40, Math.min(0.85, power));
            Tmotor.setPower(power);
            Bmotor.setPower(power);
            led.setPosition(0.500);
        } else {
            Tmotor.setPower(0);
            Bmotor.setPower(0);
            led.setPosition(tagVisible ? 0.500 : 0.277);
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
        @Override public void start() { start = System.currentTimeMillis(); gateServo.setPosition(0.97); }
        @Override public void update() { if (System.currentTimeMillis() - start > 500) gateServo.setPosition(1.0); }
        @Override public boolean isDone() { return true; }
    };
}