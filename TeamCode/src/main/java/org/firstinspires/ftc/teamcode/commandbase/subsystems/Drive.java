package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.RevIMU;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;




public class Drive {
    private final Motor FLmotor, FRmotor, BLmotor, BRmotor;
    private final DcMotorEx FLencoder, FRencoder, BLencoder, BRencoder;
    private final DcMotorEx Xdeadwheel, Ydeadwheel;
    private final RevIMU imu;

    private final PIDFController xController;
    private final PIDFController yController;
    private final PIDFController headingController;

    private final MecanumDrive drive;

    private MotorGroup driveMotor;
    private MecanumDriveOdometry odometry; //deadwheels

    private double currentX  = 0, currentY = 0, currentHeading = 0;
    private double Xdeadwheeloffset = 0, Ydeadwheeloffset = 0;
    private double Xdeadwheelmovement = 0, Ydeadwheelmovement = 0;

    public Drive(HardwareMap hwMap) {
        FLmotor = hwMap.get(Motor.class, "FLmotor");
        FRmotor = hwMap.get(Motor.class, "FRmotor");
        BLmotor = hwMap.get(Motor.class, "BLmotor");
        BRmotor = hwMap.get(Motor.class, "BRmotor");

        FLencoder = hwMap.get(DcMotorEx.class, "FLencoder");
        FRencoder = hwMap.get(DcMotorEx.class, "FRencoder");
        BLencoder = hwMap.get(DcMotorEx.class, "BLencoder");
        BRencoder = hwMap.get(DcMotorEx.class, "BRencoder");

        Xdeadwheel = hwMap.get(DcMotorEx.class, "Xdeadwheel");
        Ydeadwheel = hwMap.get(DcMotorEx.class, "Ydeadwheel");

        xController = new PIDFController(0.05, 0, 0, 0); //need to tune
        yController = new PIDFController(0.05, 0, 0, 0); //need to tune
        headingController = new PIDFController(0.01, 0, 0, 0); //need to tune

        drive = new MecanumDrive(true, FLmotor, FRmotor, BLmotor, BRmotor);

        imu = hwMap.get(RevIMU.class, "imu");
        imu.init();

    }

    public void driveTo(double targetX, double targetY, double targetHeading) {
        double xPower = xController.calculate(currentX, targetX);
        double yPower = yController.calculate(currentY, targetY);
        double headingPower = headingController.calculate(currentHeading, targetHeading);

        drive.driveRobotCentric(xPower, yPower, headingPower);
    }

    public void updateOdo() {
//        int FLticks = FLencoder.getCurrentPosition();
//        int FRticks = FRencoder.getCurrentPosition();
//        int BLticks = BLencoder.getCurrentPosition();
//        int BRticks = BRencoder.getCurrentPosition();

//        double avgencoderticks = (FLticks + FRticks + BLticks + BRticks);
//        double distancepertick = 10;//need to find later
//        double robotmovement = avgencoderticks * distancepertick;

        currentHeading = (imu.getHeading())*Math.PI/180;

        Xdeadwheelmovement = Xdeadwheel.getCurrentPosition() - Xdeadwheeloffset;
        Ydeadwheelmovement = Ydeadwheel.getCurrentPosition() - Ydeadwheeloffset;

        currentX += Xdeadwheelmovement * Math.cos(currentHeading);
        currentY += Ydeadwheelmovement * Math.sin(currentHeading);

        Xdeadwheeloffset = Xdeadwheel.getCurrentPosition();
        Ydeadwheeloffset = Ydeadwheel.getCurrentPosition();

    }

    public double getCurrentX() {
        return currentX;
    }

    public double getCurrentY() {
        return currentY;
    }

    public double getCurrentHeading() {
        return currentHeading;
    }

    public void driveTeleop(GamepadEx gamepad) {
        double forward = -gamepad.getLeftY();
        double strafe = gamepad.getLeftX();
        double rotate = gamepad.getRightX();

        double targetX = currentX + strafe;
        double targetY = currentY + forward;
        double targetHeading = currentHeading + rotate;

        driveTo(targetX, targetY, targetHeading);
    }

}