package org.firstinspires.ftc.teamcode.pedro;

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
            .mass() //need to find
            .forwardZeroPowerAcceleration() //need to find
            .lateralZeroPowerAcceleration() //need to find
            .useSecondaryTranslationalPIDF(true) //staying on path
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true) //progress in path
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.58) //need to confirm
            .rightFrontMotorName("FRmotor")
            .rightRearMotorName("BRmotor")
            .leftRearMotorName("BLmotor")
            .leftFrontMotorName("FLmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //need to confirm directions later
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity() //need to tune (automatic)
            .yVelocity(); //need to tune (automatic)

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY() //need to find
            .strafePodX() //need to find
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //need to set
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) //either 4_BAR_POD or SWINGARM, need to confirm
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD) //need to confirm
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    //check localizer tuning here: https://pedropathing.com/docs/pathing/tuning/localization#localization-test

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}