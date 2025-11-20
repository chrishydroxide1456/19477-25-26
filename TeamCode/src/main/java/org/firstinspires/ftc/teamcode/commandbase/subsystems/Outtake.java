package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}
    private final MotorEx Tmotor = new MotorEx("Tmotor").reversed();
    private final MotorEx Bmotor = new MotorEx("Bmotor");
    private final ServoEx gateServo = new ServoEx("gateServo");

    private final ControlSystem outcontroller = ControlSystem.builder()
            .velPid(0.01, 0.0, 0.0) //need to tune
            .basicFF(0.01, 0.02, 0.03) //need to tune
            .build();

    public final Command on = new RunToVelocity(outcontroller, targetVel).requires(this); //add state requirements later
    public final Command off = new RunToVelocity(outcontroller, 0.0).requires(this); //add state requirements later

    public final Command open = new SetPosition(gateServo, 1.0).requires(this);
    public final Command close = new SetPosition(gateServo, 0.0).requires(this);

    public void periodic() {
        Tmotor.setPower(outcontroller.calculate(Tmotor.getState()));
        Bmotor.setPower(outcontroller.calculate(Bmotor.getState()));
    }

}
