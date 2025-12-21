//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import dev.nextftc.hardware.impl.CRServoEx;
//import dev.nextftc.hardware.impl.ServoEx;
//
//@Config
//@TeleOp(name = "Dashboard Motor Test", group = "Test")
//public class MotorTest extends LinearOpMode {
//
//    // This value will appear in FTCDashboard under "Dashboard Motor Test"
//    public static double spinservo1power = 0.5;
//    public static double spinservo2power = 0.5;
//    public static double gateservoposition = 0.0;
//    public static double ledposition = 0.0;
//
//    private Servo gateServo;
//    private CRServo spinServo1;
//    private CRServo spinServo2;
//    private Servo led;
//    private FtcDashboard dashboard;
//
//    @Override
//    public void runOpMode() {
//        // Hardware mapping
//        gateServo = hardwareMap.get(Servo.class, "gateServo");
//        spinServo1 = hardwareMap.get(CRServo.class, "spinServo1");
//        spinServo2 = hardwareMap.get(CRServo.class, "spinServo2");
//        led = hardwareMap.get(Servo.class, "led");
//
//
//        // Optional: ensure motor stops when power is 0
//        spinServo1.setPower(0.0);
//        spinServo2.setPower(0.0);
//
//        dashboard = FtcDashboard.getInstance();
//
//        telemetry.addLine("Initialized. Open FTCDashboard to tune motorPower.");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Send the dashboard power to the motor
//            spinServo1.setPower(spinservo1power);
//            spinServo2.setPower(spinservo2power);
//            gateServo.setPosition(gateservoposition);
//            led.setPosition(ledposition);
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