package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;

import androidx.annotation.NonNull;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}
    private final MotorEx Tmotor = new MotorEx("Tmotor").reversed();
    private final MotorEx Bmotor = new MotorEx("Bmotor");
    private Servo gateServo;
    private CRServo spinServo1;
    private CRServo spinServo2;
    private Servo led;


    public void initialize() {
        gateServo.setPosition(0.0);
        led = hardwareMap.get(Servo.class, "led");
        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

    }
    private final ControlSystem outcontroller = ControlSystem.builder()
            .velPid(0.01, 0.0, 0.0) //need to tune
            .basicFF(0.01, 0.02, 0.03) //need to tune
            .build();

    //public Command on = new RunToVelocity(outcontroller, 500);
    //public Command on = new RunToVelocity(outcontroller, targetVel);
    public Command on = new Command() {

        @Override
        public void update() {
            new RunToVelocity(outcontroller, targetVel);

            if (Tmotor.getVelocity() > (targetVel - 50)) { //100 rpm = 46.67 tps
                led.setPosition(0.5);
            }
            else {
                led.setPosition(0.3);
            }
        }

        @Override
        public boolean isDone() {
            return false; //i think this is okay
        }

    };

    //public Command off = new RunToVelocity(outcontroller, 0.0);
    public Command off = new Command() {
        long t1 = System.currentTimeMillis();

        @Override
        public void update() {
            long t2 = System.currentTimeMillis();
            double dt = (double) (t2 - t1) /100;
            t1 = t2;

            if (Tmotor.getVelocity() > 0) {
                targetVel = Tmotor.getVelocity() - 400 * dt; //400 = slow-down rate
            }

            new RunToVelocity(outcontroller, targetVel);
        }

        @Override
        public boolean isDone() {
            return Bmotor.getVelocity() == 0;
        }

        public void stop(boolean interrupted) {
            new RunToVelocity(outcontroller, 0.0); //just in case
//            spinServo1.setPower(0.0);
//            spinServo2.setPower(0.0);
//            led.setPosition(0.0);
//            gateServo.setPosition(0.0);
        }

    };


    public Command open = new Command() {

        public void update() {
            gateServo.setPosition(0.2);
        }

        @Override
        public boolean isDone() {
            return gateServo.getPosition() < 0.23;
        }

        public void stop(boolean interrupted) {
            spinServo1.setPower(1.0);
            spinServo2.setPower(1.0);
        }
    };

    public Command close = new Command() {

        public void update() {
            gateServo.setPosition(1.0);
            spinServo1.setPower(0.0);
            spinServo2.setPower(0.0);
        }

        @Override
        public boolean isDone() {
            return gateServo.getPosition() > 0.95;
        }
    };

    public Command reverse = new RunToVelocity(outcontroller, -56).requires(this);


    public void periodic() {
        Tmotor.setPower(outcontroller.calculate(Tmotor.getState()));
        Bmotor.setPower(outcontroller.calculate(Bmotor.getState()));
    }

}
