package org.firstinspires.ftc.teamcode.pedroPathing;

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
            .mass(14.150)

            .forwardZeroPowerAcceleration(-28.42124519029252)
            .lateralZeroPowerAcceleration(-56.67957252937972)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.0, 0.015, 0.0225 ))
            .headingPIDFCoefficients(new PIDFCoefficients(1.4, 0, 0.002, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05, 0.00025, 0.0002, 0.6, 0.002))

            .centripetalScaling(0.00025);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)

            .rightFrontMotorName("motor_Dr_Fr")
            .rightRearMotorName("motor_Dr_A")
            .leftRearMotorName("motor_Iz_A")
            .leftFrontMotorName("motor_Iz_Fr")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(65.55825829693651)
            .yVelocity(53.18702565591167);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-110/2.54)
            .strafePodX(-160/2.54)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.4,
            4);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
