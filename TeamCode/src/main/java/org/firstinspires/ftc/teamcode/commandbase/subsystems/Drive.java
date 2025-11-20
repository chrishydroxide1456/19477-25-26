package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.MecanumDriverControlled;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private Drive() { }
    private final MotorEx FLmotor = new MotorEx("FLmotor");
    private final MotorEx FRmotor = new MotorEx("FRmotor").reversed();
    private final MotorEx BLmotor = new MotorEx("BLmotor").reversed();
    private final MotorEx BRmotor = new MotorEx("BRmotor").reversed();


    public final Command driverdrive = new MecanumDriverControlled(
            FLmotor, FRmotor, BLmotor, BRmotor,
            Gamepads.gamepad1().leftStickY(), Gamepads.gamepad1().leftStickX(), Gamepads.gamepad1().rightStickX()
    ).requires(this);

    //public final Command targetdrive = new Run

}
