package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.headingAdjust;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

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


    public void driverdrive(Gamepad gamepad) { //add multiplier later
        double y = -gamepad.left_stick_y;
        double x = -gamepad.left_stick_x * 1.1;
        double rx = -gamepad.right_stick_x;
        double FLpower = (-y + x + rx);
        double BLpower = (y + x - rx);
        double FRpower = (-y - x - rx);
        double BRpower = (y - x + rx);

        FLmotor.setPower(FLpower);
        FRmotor.setPower(FRpower);
        BLmotor.setPower(BLpower);
        BRmotor.setPower(BRpower);
    }

//    private HolonomicMode RobotCentric;
//    public final Command Driverdrive = new MecanumDriverControlled(FLmotor, FRmotor, BLmotor, BRmotor,
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX(),
//                RobotCentric
//        );
//
    public void autodrive(double rx) {
        double FLpower = (-rx);
        double BLpower = (rx);
        double FRpower = (rx);
        double BRpower = (-rx);

        FLmotor.setPower(FLpower);
        FRmotor.setPower(FRpower);
        BLmotor.setPower(BLpower);
        BRmotor.setPower(BRpower);
    }


//    private final ControlSystem drivecontroller = ControlSystem.builder()
//            .posPid(0.01, 0.0, 0.0) //need to tune
//            .basicFF(0.01, 0.02, 0.03) //need to tune
//            .build();
//
//    public Command autodrivecmd = new Command() {
//
//        public void update() {
//            drivecontroller.setGoal(new KineticState(0.0, 0.0, 0.0));
//            KineticState currentState = new KineticState(headingAdjust, 0.0, 0.0);
//            double turnPower = drivecontroller.calculate(currentState);
//            new SetPower(FLmotor, turnPower);
//            new SetPower(FRmotor, -turnPower);
//            new SetPower(BLmotor, -turnPower);
//            new SetPower(BRmotor, turnPower);
//
//        }
//
//
//        @Override
//        public boolean isDone() {
//            return Math.abs(headingAdjust) < 0.04; //like 2.3 degrees of error
//        }
//
//        public void stop() {
//            headingAdjust = 0.0;
//        }
//
//    };

}
