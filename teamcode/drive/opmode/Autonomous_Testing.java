package org.firstinspires.ftc.teamcode.drive.opmode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config

@Autonomous(group = "drive")
public class Autonomous_Testing extends LinearOpMode {

    private DcMotor LiftLift;
    private Servo LeftPincher;
    private Servo RightPincher;
    View relativeLayout;
    @Override
    public void runOpMode() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        LiftLift = hardwareMap.get(DcMotor.class, "LiftLift");
        RightPincher = hardwareMap.get(Servo.class, "RightPincher");
        LeftPincher = hardwareMap.get(Servo.class, "LeftPincher");

        LiftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftPincher.setDirection(Servo.Direction.REVERSE);
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(35.25, 63.75, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
        //To read cone
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(42, 36, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(26.5, 5, Math.toRadians(-105)), Math.toRadians(-105))
                .build();


        //Drop cone #1

        //To cone stack
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(64, 11, Math.toRadians(0)), Math.toRadians(0))

                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(25.75, 4.5, Math.toRadians(-125)), Math.toRadians(-125))
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(64, 11, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(26.5, 4, Math.toRadians(-125)), Math.toRadians(-125))
                .build();

        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(36, 12, Math.toRadians(-90)), Math.toRadians(-90))
                .build();


        //Grab top cone(cone #2 respectively)

        //Go back
       // Trajectory traj10 = drive.trajectoryBuilder(traj3.end())
         //       .back(39)
          //      .build();

        //Turn
        Trajectory traj11 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(7.5)
                .build();

        //Drop

        //Go back
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .back(7.5)
                .build();
        //turn
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(40)
                .build();
        Trajectory traj14 = drive.trajectoryBuilder(traj13.end())
                .back(38)
                .build();

        Trajectory traj15 = drive.trajectoryBuilder(traj14.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(4.5)
                .build();


        waitForStart();
        //Go to color sensor read
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        sleep(275);
        //lift to pole
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2875);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj1);


       // drive.followTrajectorySequence(traj2);
        sleep(400);
   //Release and run spline to stack
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(500);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(325);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj2);
        sleep(50);
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        sleep(250);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2875);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        drive.followTrajectorySequence(traj3);
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(500);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(250);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj4);
        sleep(50);
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        sleep(350);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2875);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj5);
//release
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(300);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(0);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        drive.followTrajectorySequence(traj6);
        /*


        drive.followTrajectory(traj12);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(250);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.turn(Math.toRadians(90));
        sleep(500);

        drive.followTrajectory(traj13);
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        sleep(500);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(750);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        drive.followTrajectory(traj14);
        drive.turn(Math.toRadians(-90));
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2950);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        drive.followTrajectory(traj15);
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(500);
*/
    }
}