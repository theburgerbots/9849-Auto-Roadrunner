package org.firstinspires.ftc.teamcode.drive.opmode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
        Pose2d startPose = new Pose2d(36, 63.5, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
//To read cone
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(39, 48), Math.toRadians(-90))
                .build();
        //To push read cone out of the way
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(36, 2.5), Math.toRadians(-90))
                .build();
        //back to post level
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(7)
                .build();
        //Strafe to in front of post
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(13)
                .build();
        //forward

        //lift lift

        //creep forward
        Trajectory traj6 = drive.trajectoryBuilder(traj4.end())
                .forward(4.5)
                .build();
        //drop
        //wait

        //Now go back
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(7)
                .build();
        //TURN

        //Strafe to correct
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .strafeRight(5)
                .build();

        //Go to stack
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(40)
                .build();

        //GRab

        //Go back
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .back(40)
                .build();

        //Turn
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(4.5)
                .build();

        //Drop

        //Go back
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .back(7)
                .build();
        //turn
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(39)
                .build();
        waitForStart();
        //Go to color sensor read
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
       drive.followTrajectory(traj4);


        //lift to pole
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2950);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);

        drive.followTrajectory(traj6);
        //release
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        drive.followTrajectory(traj7);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(0);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);

        drive.turn(Math.toRadians(90));

        drive.followTrajectory(traj8);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(350);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(traj9);

        sleep(500);
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        sleep(550);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(1000);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(750);
        drive.followTrajectory(traj10);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2950);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj11);
        //release
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(500);
        drive.followTrajectory(traj12);
        drive.turn(Math.toRadians(90));
        sleep(500);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(350);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectory(traj13);

    }
}