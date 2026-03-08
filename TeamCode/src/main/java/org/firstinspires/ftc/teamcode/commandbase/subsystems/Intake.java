package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}
    public final MotorEx inmotor1 = new MotorEx("inmotor1").reversed();
    public final MotorEx inmotor2 = new MotorEx("inmotor2"); //reversed or not?

    MotorGroup inmotors = new MotorGroup(inmotor1, inmotor2);

    public Command on = new SetPower(inmotors, 1.0).requires(this);
    public Command onmoving = new SetPower(inmotors, 0.9);//0.65 TODO:Redo this if it fails
    public Command autonshooting = new SetPower(inmotors, 0.70);
    public Command revmoving = new SetPower(inmotors, -0.9);
    public Command keeping = new SetPower(inmotor1, 0.8);
    public Command off = new SetPower(inmotors, 0.0).requires(this);
    public Command off2 = new SetPower(inmotor2, 0.0);
    public Command reverse = new SetPower(inmotors, -1.0).requires(this);

}