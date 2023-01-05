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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config

@Autonomous(group = "drive")
public class Left_3_Cone_With_Read extends LinearOpMode {
    private ColorSensor colorSensor_REV_ColorRangeSensor;
    private DcMotor LiftLift;
    private Servo LeftPincher;
    private Servo RightPincher;
    int Parkacola = 0;
    int AntiParkacola;
    boolean Read;
    NormalizedColorSensor colorSensor;
    View relativeLayout;

    @Override

    public void runOpMode() throws InterruptedException {

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        LiftLift = hardwareMap.get(DcMotor.class, "LiftLift");
        RightPincher = hardwareMap.get(Servo.class, "RightPincher");
        LeftPincher = hardwareMap.get(Servo.class, "LeftPincher");


        try {
            runSample();
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    protected void runSample() throws InterruptedException {

        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        boolean bPrevState = false;
        boolean bCurrState = false;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(35.25, 63.75, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
        //To read cone
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(42, 38, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(27.5, 6, Math.toRadians(-125)), Math.toRadians(-125))
                .build();
        //Drop cone #1

        //To cone stack
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(63.2, 11.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        //To pole drop cone #2
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(27, 2.5, Math.toRadians(-125)), Math.toRadians(-125))
                .build();
        //To cone stack for 3rd cone
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(63.5, 10.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        // To pole to drop cone 3
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(27.7, 1.5, Math.toRadians(-125)), Math.toRadians(-125))
                .build();

// Start Park trajectories

        // park traj 1
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj6.end())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(60, 12.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        // park traj 2
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj6.end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(36, 12, Math.toRadians(0)), Math.toRadians(-90))
                .build();
        // park traj 3
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj6.end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)), Math.toRadians(-90))
                .build();






        LiftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftPincher.setDirection(Servo.Direction.REVERSE);
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        waitForStart();


        LeftPincher.setPosition(.17);  //close claw
        RightPincher.setPosition(.14);  // close claw
        sleep(500); // Wait time to close


        if (bCurrState != bPrevState) {
            if (bCurrState) {
                if (colorSensor instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight) colorSensor;
                    light.enableLight(!light.isLightOn());
                }
            }
        }
        bPrevState = bCurrState;


        //Start Trajectories
        telemetry.update();
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2875);   //lift to high junction height
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj1); //Drive to read cone and push it left
        //Color sensor read
        while (!(Parkacola > 0)) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            AntiParkacola++;
            sleep(1);
            if(AntiParkacola > 1000){
                Parkacola = 2;
            }


            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.update();

            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            if (colors.red > .3 && colors.red < .5 && colors.blue < .750 && colors.blue > .650) {
                Parkacola = 1;
                telemetry.addData("Color: ", "Black==park#1");
                telemetry.addData("Parkacola", Parkacola);
                telemetry.update();
                Read = true;
            } else if (colors.red > .3 && colors.red < .5  && colors.blue > .75) {
                Parkacola = 2;
                telemetry.addData("Color: ", "Blue==Park#2");
                telemetry.addData("Parkacola", Parkacola);
                telemetry.update();
                Read = true;
            } else if (colors.red > .5 && colors.blue <.75) {
                Parkacola = 3;
                telemetry.addData("Color: ", "Red==#3");
                telemetry.addData("Parkacola", Parkacola);
                telemetry.update();
                Read = true;

            }
        }

        drive.followTrajectorySequence(traj2);
        sleep(400);
        //Release and run spline to stack
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(500);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(325);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj3);
        sleep(50);
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        sleep(250);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2875);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        drive.followTrajectorySequence(traj4);
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(500);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(250);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj5);
        sleep(50);
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        sleep(350);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2875);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(traj6);
//release
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);
        sleep(300);
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(0);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);



            //Green Park
            if (Parkacola == 1) {
                drive.followTrajectorySequence(park1);
            }
            //Pink Park
            if (Parkacola == 2) {
                drive.followTrajectorySequence(park2);
            }
            //Yellow Park
            if (Parkacola == 3) {
                drive.followTrajectorySequence(park3);
           }
    }
}
