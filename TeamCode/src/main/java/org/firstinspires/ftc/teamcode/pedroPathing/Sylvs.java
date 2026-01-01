package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
// import com.pedropathing.geometry.Point;

@Autonomous
public class Sylvs extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private Pose startPose, lanz1, rec1, finrec1, lanz2, rec2, finrec2, finPose, meantime, rec3, meantime2;
    private PathChain cam1, cam2, cam3, cam4, cam5, cam6, cam7, cam8, curvs, prueba, ccam1;





    /*
    cam1 : startPose - lanz1 *
    cam2 : lanz1 - rec1
    cam3 : rec1 - finrec1
    prueba : curva finrec1 - lanz2 *
    cam5 : lanz2 - rec2
    cam6 : rec2 - finrec2
    cam7 : finrec2 - lanz2 *
    cam8 : lanz2 - finPose
     */

    @Override
    public void runOpMode() {

        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limaluz");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(7); // Switch to pipeline number 0

        limelight.start(); // This tells Limelight to start looking!


        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        // Definir poses
        startPose = new Pose(23, 142, Math.toRadians(0));
        lanz1 = new Pose(8, 0, Math.toRadians(25));
        meantime = new Pose(30, 90, Math.toRadians(90));
        /*
        rec1 = new Pose(30, 10, Math.toRadians(90));
        finrec1 = new Pose(30, 40, Math.toRadians(90));
        lanz2 = new Pose(75, 0, Math.toRadians(45));
        rec2 = new Pose(48, 10, Math.toRadians(90));
        finrec2 = new Pose(48, 40, Math.toRadians(90));
        finPose = new Pose(55, 3.5, Math.toRadians(0));

        meantime = new Pose(30, -10, Math.toRadians(90));
        meantime2 = new Pose(30, 0, Math.toRadians(90));
        rec3 = new Pose(66, 10, Math.toRadians(90));
        */
        // Construir paths



        curvs = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, meantime2, rec1))
                //.setHeadingInterpolation(HeadingInterpolator.facingPoint(0, 90))
                .build();
        follower.followPath(curvs);
    /*
        prueba = follower.pathBuilder()
                .addPath(new BezierCurve(finrec1, meantime, lanz2))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(15, 90))
                .build();
        follower.followPath(prueba);

        ccam1 = follower.pathBuilder()
                .addPath(new BezierLine(finrec1, rec1))
                .setLinearHeadingInterpolation(finrec1.getHeading(), rec1.getHeading())
                .build();

        cam1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lanz1))
                .setLinearHeadingInterpolation(startPose.getHeading(), lanz1.getHeading())
                .build();

        cam2 = follower.pathBuilder()
                .addPath(new BezierLine(lanz1, rec1))
                .setLinearHeadingInterpolation(lanz1.getHeading(), rec1.getHeading())
                .build();

        cam3 = follower.pathBuilder()
                .addPath(new BezierLine(rec1, finrec1))
                .setLinearHeadingInterpolation(rec1.getHeading(), finrec1.getHeading())
                .build();

        cam4 = follower.pathBuilder()
                .addPath(new BezierLine(finrec1, lanz2))
                .setLinearHeadingInterpolation(finrec1.getHeading(), lanz2.getHeading())
                .build();

        cam5 = follower.pathBuilder()
                .addPath(new BezierLine(lanz2, rec2))
                .setLinearHeadingInterpolation(lanz2.getHeading(), rec2.getHeading())
                .build();

        cam6 = follower.pathBuilder()
                .addPath(new BezierLine(rec2, finrec2))
                .setLinearHeadingInterpolation(rec2.getHeading(), finrec2.getHeading())
                .build();

        cam7 = follower.pathBuilder()
                .addPath(new BezierLine(finrec2, lanz2))
                .setLinearHeadingInterpolation(finrec2.getHeading(), lanz2.getHeading())
                .build();

        cam8 = follower.pathBuilder()
                .addPath(new BezierLine(lanz2, finPose))
                .setLinearHeadingInterpolation(lanz2.getHeading(), finPose.getHeading())
                .build();

*/
        follower.setStartingPose(startPose);

        waitForStart();

        LLResult result = limelight.getLatestResult();
        int ID = -2;



        sleep(1000);
        pathState = 0;
        pathTimer.resetTimer();



        while (opModeIsActive()) {

            limelight.pipelineSwitch(6);



            telemetry.update();

            follower.update();
            switch (pathState) {
                case 0:
                    follower.followPath(cam1);
                    pathState = 1;
                    telemetry.addLine("a");
                    telemetry.update();
                    break;

                default:
                    // No hacer nada
                    break;
            }


            // Telemetría (opcional)
            /*
            telemetry.addData("state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();


             */
        }

    }
}

/*
public class Sylvs extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private Pose startPose, lanz1, rec1, finrec1, lanz2, rec2, finrec2, finPose;
    private PathChain cam1, cam2, cam3, cam4, cam5, cam6, cam7, cam8;

    /*
    cam1 : startPose - lanz1 *
    cam2 : lanz1 - rec1
    cam3 : rec1 - finrec1
    cam4 : finrec1 - lanz2 *
    cam5 : lanz2 - rec2
    cam6 : rec2 - finrec2
    cam7 : finrec2 - lanz2 *
    cam8 : lanz2 - finPose


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        // Definir poses
        startPose  = new Pose(0, 0, Math.toRadians(0));
        lanz1  = new Pose(8,   0,  Math.toRadians(45));
        rec1    = new Pose(30,  10,  Math.toRadians(90));
        finrec1    = new Pose(30,  40,  Math.toRadians(90));
        lanz2    = new Pose(95,  5,  Math.toRadians(45));
        rec2  = new Pose(48,   10,  Math.toRadians(90));
        finrec2    = new Pose(48,  40,  Math.toRadians(90));
        finPose = new Pose(55,  3.5,  Math.toRadians(0));

        // Construir paths

        cam1   = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lanz1))
                .setLinearHeadingInterpolation(startPose.getHeading(), lanz1.getHeading())
                .build();

        cam2  = follower.pathBuilder()
                .addPath(new BezierLine(lanz1, rec1))
                .setLinearHeadingInterpolation(lanz1.getHeading(), rec1.getHeading())
                .build();

        cam3   = follower.pathBuilder()
                .addPath(new BezierLine(rec1, finrec1))
                .setLinearHeadingInterpolation(rec1.getHeading(), finrec1.getHeading())
                .build();

        cam4  = follower.pathBuilder()
                .addPath(new BezierLine(finrec1, lanz2))
                .setLinearHeadingInterpolation(finrec1.getHeading(), lanz2.getHeading())
                .build();

        cam5   = follower.pathBuilder()
                .addPath(new BezierLine(lanz2, rec2))
                .setLinearHeadingInterpolation(lanz2.getHeading(), rec2.getHeading())
                .build();

        cam6  = follower.pathBuilder()
                .addPath(new BezierLine(rec2, finrec2))
                .setLinearHeadingInterpolation(rec2.getHeading(), finrec2.getHeading())
                .build();

        cam7  = follower.pathBuilder()
                .addPath(new BezierLine(finrec2, lanz2))
                .setLinearHeadingInterpolation(finrec2.getHeading(), lanz2.getHeading())
                .build();

        cam8  = follower.pathBuilder()
                .addPath(new BezierLine(lanz2, finPose))
                .setLinearHeadingInterpolation(lanz2.getHeading(), finPose.getHeading())
                .build();


        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        switch (pathState) {
            case 0:
                follower.followPath(cam1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(cam2, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(cam3, true);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(cam4, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(cam5, true);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(cam6, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(cam7, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(cam8, true);
                    pathState = 8;
                }
                break;
            case 8:
                // Auto terminado
                break;
            default:
                // No hacer nada
                break;
        }



        // Telemetría (opcional)
        telemetry.addData("state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}

*/