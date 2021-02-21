/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


@Config
@Autonomous(name = "TylerAutonomousRed", group = "Autonomous")
//@Disabled
public class TylerAutonomousRed extends LinearOpMode {
    public double timeScanning = 2; //time spent trying to find the highest confidence recognition
    public double distanceToMoveForwardToShoot = 72;
    public Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(0)); //starting outside Blue Alliance

    //Motor and Servo constants
    public double FIRE_STANDBY_SERVO = 0.77;   // Position for the servo to be in when not firing
    public double FIRE_SERVO = 1;         // Position for the servo when firing a ring
    public double CLAW_STANDBY_SERVO = 0;
    public double CLAW_GRAB = 1;
    public int WOBBLE_GRAB = -900;     // Position for grabbing the wobble goal off the field
    public int WOBBLE_OVER_WALL = -450; // Position for raising the wobble goal over the wall
    public int WOBBLE_CARRY = -630;     // POsition for carrying the wobble goal to the wall
    public int HIGH_GOAL_VEL = 1220;
    public int MID_GOAL_VEL = 1150;

    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AakkMZL/////AAABmRnl+IbXpU2Bupd2XoDxqmMDav7ioe6D9XSVSpTJy8wS6zCFvTvshk61FxOC8Izf/oEiU7pcan8AoDiUwuGi64oSeKzABuAw+IWx70moCz3hERrENGktt86FUbDzwkHGHYvc/WgfG3FFXUjHi41573XUKj7yXyyalUSoEbUda9bBO1YD6Veli1A4tdkXXCir/ZmwPD9oA4ukFRD351RBbAVRZWU6Mg/YTfRSycyqXDR+M2F/S8Urb93pRa5QjI4iM5oTu2cbvei4Z6K972IxZyiysbIigL/qjmZHouF9fRO4jHoJYzqVpCVYbBVKvVwn3yZRTAHf9Wf77/JG5hJvjzzRGoQ3OHMt/Ch93QbnJ7zN";

    //Vuforia Localizer which will produce frames for TFOD
    private VuforiaLocalizer vuforia;

    //TFOD engine
    private TFObjectDetector tfod;

    public DcMotorEx ringShooterMotor = null;
    public DcMotorEx wobbleMotor = null;
    public DcMotor intakeMotor = null;

    public Servo fireServo = null;
    public Servo clawServo = null;


    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(startingPose);

        VinceHardwareBruinBot robot = new VinceHardwareBruinBot();
        robot.init(hardwareMap);
        {
            intakeMotor = robot.intakeMotor;
            wobbleMotor = robot.wobbleMotor;
            ringShooterMotor = robot.ringShooterMotor;

            fireServo = robot.fireServo;
            clawServo = robot.clawServo;
        }

        // Reset the wobble motor - Use a repeatable starting position
        //wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 1.78);
        }

        /** Wait for the game to begin */
        //telemetry.addData(">", "Press Play to start op mode");
        //telemetry.update();
/*
        Trajectory driveForward = drive.trajectoryBuilder(startingPose)
                .forward(50)
                .build();

        Trajectory strafe = drive.trajectoryBuilder(driveForward.end())
                .forward(18)
                .build();

        Trajectory driveForward2 = drive.trajectoryBuilder(driveForward.end())
                .forward(22)
                .build();

        //-------------------------------------------------------
        //1st

        Trajectory back = drive.trajectoryBuilder(driveForward2.end())
                .back(10)
                .build();

        Trajectory unBack = drive.trajectoryBuilder(back.end())
                .forward(10)
                .build();

        Trajectory back10 = drive.trajectoryBuilder(unBack.end())
                .back(10)
                .build();
*/


        double highestConfidence = 0;
        String highestConfidenceLabel = "None";

        while (!isStarted()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    System.out.println(updatedRecognitions);
                    /*for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getConfidence() > highestConfidence) {
                            highestConfidence = recognition.getConfidence();
                            highestConfidenceLabel = recognition.getLabel();
                        }
                    }*/

                    if (updatedRecognitions.size() > 0) {
                        highestConfidence = updatedRecognitions.get(0).getConfidence();
                        highestConfidenceLabel = updatedRecognitions.get(0).getLabel();
                    }
                    else
                        highestConfidenceLabel = "None";
                }
            }
            telemetry.addData("caption", highestConfidenceLabel);
            telemetry.addData("confidence", highestConfidence);
            telemetry.update();

        }

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        fireServo.setPosition(FIRE_STANDBY_SERVO);
        //clawServo.setPosition(CLAW_GRAB);

        runtime.reset();

        if (opModeIsActive() && !isStopRequested()) {
            sleep(4000);
            /*telemetry.addData("time", runtime.time());
            telemetry.addData("Label", highestConfidenceLabel);
            telemetry.addData("Confidence", highestConfidence);
            telemetry.update();*/

                /*drive.followTrajectory(driveForward);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(strafe);
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(driveForward2);
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(back);

                intakeMotor.setPower(0.3);
                sleep(1000);
                intakeMotor.setPower(0);

                drive.followTrajectory(unBack);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(back10);

                ringShooterMotor.setVelocity(HIGH_GOAL_VEL);

                sleep(2000);

                //start shooting rings
                for (int i = 0; i < 3; i++) {
                    fireServo.setPosition(FIRE_SERVO);
                    sleep(1000);
                    fireServo.setPosition(FIRE_STANDBY_SERVO);
                    sleep(2000);
                }

                ringShooterMotor.setPower(0);*/


            switch (highestConfidenceLabel) {
                case "None":
                    box1();
                    break;
                case "Single":
                    box2();
                    break;
                case "Quad":
                    box3();
                    break;
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void box1() {
        Trajectory box1Traj = drive.trajectoryBuilder(startingPose)
                .splineTo(new Vector2d(77, 27), Math.toRadians(-90))
                //.splineTo(new Vector2d(-12, 84), Math.toRadians(0))
                .build();

        Trajectory box1ShootTraj = drive.trajectoryBuilder(box1Traj.end())
                .splineTo(new Vector2d(55, 15), Math.toRadians(179))
                .build();

        Trajectory getToLineBox1 = drive.trajectoryBuilder(box1ShootTraj.end())
                .forward(20)
                .build();

        drive.followTrajectory(box1Traj);

        intakeMotor.setPower(0.25);
        sleep(1000);
        intakeMotor.setPower(0);

        drive.followTrajectory(box1ShootTraj);
        drive.turn(Math.toRadians(180));

        ringShooterMotor.setVelocity(HIGH_GOAL_VEL);

        sleep(2000);

        //start shooting rings
        for (int i = 0; i < 3; i++) {
            fireServo.setPosition(FIRE_SERVO);
            sleep(500);
            fireServo.setPosition(FIRE_STANDBY_SERVO);
            sleep(500);
        }

        ringShooterMotor.setPower(0);

        drive.followTrajectory(getToLineBox1);
    }

    private void box2() {
        Trajectory box2Traj = drive.trajectoryBuilder(startingPose)
                .splineTo(new Vector2d(90, 15), 0)
                .build();

        Trajectory box2ShootTraj = drive.trajectoryBuilder(box2Traj.end())
                .forward(30)
                .build();

        Trajectory getToLineBox2 = drive.trajectoryBuilder(box2ShootTraj.end())
                .forward(10)
                .build();

        drive.followTrajectory(box2Traj);

        drive.turn(Math.toRadians(180));

        intakeMotor.setPower(0.25);
        sleep(1000);
        intakeMotor.setPower(0);

        drive.followTrajectory(box2ShootTraj);

        drive.turn(Math.toRadians(-180));

        ringShooterMotor.setVelocity(HIGH_GOAL_VEL);

        sleep(2000);

        //start shooting rings
        for (int i = 0; i < 3; i++) {
            fireServo.setPosition(FIRE_SERVO);
            sleep(500);
            fireServo.setPosition(FIRE_STANDBY_SERVO);
            sleep(500);
        }

        ringShooterMotor.setPower(0);

        drive.followTrajectory(getToLineBox2);
    }

    private void box3() {
        Trajectory box3Traj = drive.trajectoryBuilder(startingPose)
                .splineTo(new Vector2d(124, 27), Math.toRadians(-90))
                //.splineTo(new Vector2d(-12, 84), Math.toRadians(0))
                .build();

        Trajectory box3ShootTraj = drive.trajectoryBuilder(box3Traj.end())
                .splineTo(new Vector2d(55, 26), Math.toRadians(181))
                .build();

        Trajectory getToLineBox3 = drive.trajectoryBuilder(box3ShootTraj.end())
                .forward(10)
                .build();

        drive.followTrajectory(box3Traj);

        intakeMotor.setPower(0.25);
        sleep(1000);
        intakeMotor.setPower(0);

        drive.followTrajectory(box3ShootTraj);
        drive.turn(Math.toRadians(185));

        ringShooterMotor.setVelocity(HIGH_GOAL_VEL);

        sleep(2000);

        //start shooting rings
        for (int i = 0; i < 3; i++) {
            fireServo.setPosition(FIRE_SERVO);
            sleep(500);
            fireServo.setPosition(FIRE_STANDBY_SERVO);
            sleep(800);
        }

        ringShooterMotor.setPower(0);

        drive.followTrajectory(getToLineBox3);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
