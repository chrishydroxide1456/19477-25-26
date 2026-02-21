//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import dev.nextftc.hardware.impl.CRServoEx;
//import dev.nextftc.hardware.impl.MotorEx;
//import dev.nextftc.hardware.impl.ServoEx;
//
//@TeleOp(name = "Dashboard Motor Test", group = "Test")
//public class MotorTest extends LinearOpMode {
//
//    // This value will appear in FTCDashboard under "Dashboard Motor Test"
//    public static double spinservo1power = 1.0;
//    public static double spinservo2power = 1.0;
//    public static double gateservoposition = 0.0;
//    public static double ledposition = 0.0;
//
//    private Servo gateServo;
//    private MotorEx Tmotor;
//    private MotorEx Bmotor;
//    private Servo led;
//    private double Tmotorvel;
//    private double Bmotorvel;
//
//    @Override
//    public void runOpMode() {
//        // Hardware mapping
//        gateServo = hardwareMap.get(Servo.class, "gateServo");
////        Tmotor = new MotorEx("Tmotor").reversed();
////        Bmotor = new MotorEx("Bmotor");
//        led = hardwareMap.get(Servo.class, "led");
//
//        telemetry.addLine("Initialized. Open FTCDashboard to tune motorPower.");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Send the dashboard power to the motor
//            Tmotor.setPower(spinservo1power);
//            Bmotor.setPower(spinservo2power);
//            gateServo.setPosition(gateservoposition);
//            led.setPosition(ledposition);
//
//            Tmotorvel = Tmotor.getVelocity();
//            Bmotorvel = Bmotor.getVelocity();
//
//            // Telemetry to driver station
//            telemetry.addData("spinservo1", spinservo1power);
//            telemetry.addData("spinservo2", spinservo2power);
//            telemetry.addData("gateservo", gateservoposition);
//            telemetry.addData("led", ledposition);
//            telemetry.update();
//        }
//    }
//}