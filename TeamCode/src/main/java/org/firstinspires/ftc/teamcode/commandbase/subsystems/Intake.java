package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}
    private final MotorEx inmotor = new MotorEx("inmotor");

    public final Command on = new SetPower(inmotor, 1.0);
    public final Command off = new SetPower(inmotor, 0.0);
    public final Command reverse = new SetPower(inmotor, -1.0);

}
