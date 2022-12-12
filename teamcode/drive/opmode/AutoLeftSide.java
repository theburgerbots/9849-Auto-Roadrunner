package org.firstinspires.ftc.teamcode.drive.opmode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Config

@Autonomous(group = "drive")
public class AutoLeftSide extends LinearOpMode {
    private ColorSensor colorSensor_REV_ColorRangeSensor;
    private DcMotor LiftLift;
    private Servo LeftPincher;
    private Servo RightPincher;
    int Parkacola=0;
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
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(36, 65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);


        //Order of operations here
        //
        //
        //Trajectory List
        //
        //


        //Spline to cone
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(40, 48), Math.toRadians(-90))
                .build();

        //Push cone forward 60 out of the way
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(40, 48, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(36, 4), Math.toRadians(-90))
                .build();

        //COme back
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(36, 4, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(-90))
                .build();

        //Drift Right
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(36, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(24, 12), Math.toRadians(-90))
                .build();

        //lift

        //creep to pole
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(24, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(24, 3), Math.toRadians(-90))
                .build();

        //release

        //creep backwards
        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(24, 3, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(24, 12), Math.toRadians(-90))
                .build();

        //lower lift

        //park
        //Green
        Trajectory traj13 = drive.trajectoryBuilder(new Pose2d(24, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(60, 12), Math.toRadians(-90))
                .build();
        Trajectory traj14 = drive.trajectoryBuilder(new Pose2d(60, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(60, 36), Math.toRadians(-90))
                .build();
        //Pink
        Trajectory traj15 = drive.trajectoryBuilder(new Pose2d(24, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(-90))
                .build();
        Trajectory traj16 = drive.trajectoryBuilder(new Pose2d(36, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(36, 36), Math.toRadians(-90))
                .build();
        //Yellow
        Trajectory traj17 = drive.trajectoryBuilder(new Pose2d(24, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(-90))
                .build();
        Trajectory traj18 = drive.trajectoryBuilder(new Pose2d(12, 12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(-90))
                .build();



        LiftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftPincher.setDirection(Servo.Direction.REVERSE);
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);

        waitForStart();
        if (bCurrState != bPrevState) {
            if (bCurrState) {
                if (colorSensor instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight)colorSensor;
                    light.enableLight(!light.isLightOn());
                }
            }
        }
        bPrevState = bCurrState;

        telemetry.update();

        //Go to color sensor read
        LeftPincher.setPosition(.16);
        RightPincher.setPosition(.14);
        drive.followTrajectory(traj1);



        //Color sensor read
        while(!(Parkacola > 0)){
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

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
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.update();

            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            if(colors.red >= .47 && colors.blue >= .76 && colors.blue < .78 && colors.red <= .49){
                Parkacola=1;
                telemetry.addData("Color: ", "Green");
                telemetry.addData("Parkacola", Parkacola);
                telemetry.update();
                Read = true;
            }
            else if(colors.red >= .49 && colors.blue >= .74 && colors.blue < .75 && colors.red < .50){
                Parkacola=3;
                telemetry.addData("Color: " , "Yellow");
                telemetry.addData("Parkacola", Parkacola);
                telemetry.update();
                Read = true;
            }
            else if(colors.red >= .49 && colors.blue >= .78){
                Parkacola=2;
                telemetry.addData("Color: ", "Pink");
                telemetry.addData("Parkacola", Parkacola);
                telemetry.update();
                Read = true;

            }
        }

        sleep(1000);
        //Push cone forward out of the way
        drive.followTrajectory(traj2);

        //Come back from cone
        drive.followTrajectory(traj3);

        //drift right to pole
        drive.followTrajectory(traj4);

        //lift to pole
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(2950);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);

        //creep to pole
        drive.followTrajectory(traj5);

        //release
        LeftPincher.setPosition(0);
        RightPincher.setPosition(0);

        //creep back
        drive.followTrajectory(traj6);

        //lower lift
        LiftLift.setPower(1);
        LiftLift.setTargetPosition(0);
        LiftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //skip to park or continue

        //turn left towards cone stack (start of repeat area) 7

        //lift to cone stack

        //go forward towards cone stack 8

        //backwards to pole 9

        //turn right towards pole 10

        //lift to pole

        //creep forward 11

        //release

        //creep backwards (End of repeat area) 12

        //Green Park
        if (Parkacola == 1){
            drive.followTrajectory(traj13);
            drive.followTrajectory(traj14);
        }

        //Pink Park
        if (Parkacola == 2){
            drive.followTrajectory(traj15);
            drive.followTrajectory(traj16);
        }

        //Yellow Park
        if (Parkacola == 3){
            drive.followTrajectory(traj17);
            drive.followTrajectory(traj18);
        }



        /*

        //GO to stack of cones
        drive.followTrajectory(traj4);

        //creep to pole
        drive.followTrajectory(traj5);
         */

    }
}
