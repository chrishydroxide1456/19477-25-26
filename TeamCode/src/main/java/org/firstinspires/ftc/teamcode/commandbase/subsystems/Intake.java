package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}
    private final MotorEx inmotor = new MotorEx("inmotor").reversed();

    public Command on = new SetPower(inmotor, 0.4).requires(this);
    public Command off = new SetPower(inmotor, 0.0).requires(this);
    public Command reverse = new SetPower(inmotor, -0.4).requires(this);

}
