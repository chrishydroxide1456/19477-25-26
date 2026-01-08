package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.0)
            .forwardZeroPowerAcceleration(-34.790656451320785)
            .lateralZeroPowerAcceleration(-79.36666688342288)
            .centripetalScaling(0.0003)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.0, 0.01, 0.035))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0.0, 0.001, 0.03)) //0.035
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0.0, 0.000005, 0.6, 0.0)) //no idea what t is what it is but everyone uses 0.6
//            .useSecondaryTranslationalPIDF(true) //staying on path
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true) //progress in path
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //need to confirm
            .rightFrontMotorName("FRmotor")
            .rightRearMotorName("BRmotor")
            .leftRearMotorName("BLmotor")
            .leftFrontMotorName("FLmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //need to confirm directions later
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(56.3066329)
            .yVelocity(38.216766357);
            //.useBrakeModeInTeleOp(false); //true or false??

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //is this set?
            .forwardPodY(1.2) //could be more accurate
            .strafePodX(-4.48) //could be more accurate
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    //run turn tuner to find yawscalar later


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            0.95,
            1.2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}