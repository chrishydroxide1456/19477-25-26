package org.firstinspires.ftc.teamcode.commandbase.subsystems;



import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.HolonomicMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.driving.RobotCentric;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private Drive() { }
    private final MotorEx FLmotor = new MotorEx("FLmotor");
    private final MotorEx FRmotor = new MotorEx("FRmotor").reversed();
    private final MotorEx BLmotor = new MotorEx("BLmotor").reversed();
    private final MotorEx BRmotor = new MotorEx("BRmotor").reversed();


    public void driverdrive(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = -gamepad.right_stick_x;
        double FLpower = (y - x + rx);
        double BLpower = (y + x + rx);
        double FRpower = (y - x - rx);
        double BRpower = (y + x - rx);

        FLmotor.setPower(FLpower);
        FRmotor.setPower(FRpower);
        BLmotor.setPower(BLpower);
        BRmotor.setPower(BRpower);
    }

    private HolonomicMode RobotCentric;
    public final Command Driverdrive = new MecanumDriverControlled(FLmotor, FRmotor, BLmotor, BRmotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                RobotCentric
        );

    public void autodrive(double y, double x, double rx) {
        double FLpower = (y + x + 0.03 * rx);
        double BLpower = (y - x + 0.03 * rx);
        double FRpower = (y - x - 0.03 * rx);
        double BRpower = (y + x - 0.03 * rx);

        FLmotor.setPower(FLpower);
        FRmotor.setPower(FRpower);
        BLmotor.setPower(BLpower);
        BRmotor.setPower(BRpower);
    } //make this a class or command?

}
