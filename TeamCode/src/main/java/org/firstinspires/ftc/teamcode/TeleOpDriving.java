/* Copyright (c) 2017 FIRST. All rights reserved.
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

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Driving", group="Iterative Opmode")
//@Disabled
public class TeleOpDriving extends OpMode
{
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;

    public BNO055IMU imu = null;

    //how fast the max power is for drive wheels
    double power = 0.3; //used to control max drive power

    //variables to maintain a heading
    public double previousHeading = 0;
    public double deadband = toRadians(3);

    //variables for object detection
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AakkMZL/////AAABmRnl+IbXpU2Bupd2XoDxqmMDav7ioe6D9XSVSpTJy8wS6zCFvTvshk61FxOC8Izf/oEiU7pcan8AoDiUwuGi64oSeKzABuAw+IWx70moCz3hERrENGktt86FUbDzwkHGHYvc/WgfG3FFXUjHi41573XUKj7yXyyalUSoEbUda9bBO1YD6Veli1A4tdkXXCir/ZmwPD9oA4ukFRD351RBbAVRZWU6Mg/YTfRSycyqXDR+M2F/S8Urb93pRa5QjI4iM5oTu2cbvei4Z6K972IxZyiysbIigL/qjmZHouF9fRO4jHoJYzqVpCVYbBVKvVwn3yZRTAHf9Wf77/JG5hJvjzzRGoQ3OHMt/Ch93QbnJ7zN";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        HardwareBruinBot robot = new HardwareBruinBot();

        robot.init(hardwareMap);

        //init all drive wheels
        leftFrontDrive = robot.leftFrontDrive;
        rightFrontDrive = robot.rightFrontDrive;
        leftRearDrive = robot.leftRearDrive;
        rightRearDrive = robot.rightRearDrive;

        //init imu
        imu = robot.imu;

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

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;

        //get input from controller
        double x  = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        //find the difference between the heading and where we want to go
        double correctedAngle = getError(Math.atan2(y, x));

        //adjust rotation parameter to spin opposite to rotation drift
        if (r != 0)
            previousHeading = getHeading(); //makes sure that while intentionally spinning no correction is being made
        else
            r += counterspin();

        double originalMagnitude = Math.hypot(y, x); //how far the joystick is being pressed so that power can be scaled
        double correctedX = Math.cos(correctedAngle) * originalMagnitude; //break it back up to send to movebot
        double correctedY = Math.sin(correctedAngle) * originalMagnitude;

        //double[] wheelSpeeds = moveBot(x, r, y, power); //to control from a robot perspective
        moveBot(correctedX, r, correctedY, power); //to control from a field perspective


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("controllerX", "(%.2f)", gamepad1.left_stick_x);
        telemetry.addData("controllerY", "(%.2f)", -gamepad1.left_stick_y);
        telemetry.addData("rotation", "(%.2f)", r);


        telemetry.addData("desiredAngle", "(%.2f)", toDegrees(Math.atan2(y, x)));
        telemetry.addData("gyroAngle", "(%.2f)", toDegrees(getHeading()));
        telemetry.addData("correctedAngled", "(%.2f)", toDegrees(correctedAngle));
        telemetry.addData("Trying to correct", "(%.2f)", counterspin());
        telemetry.addData("Error from last desired", "(%.2f)", toDegrees(getError(previousHeading)));

        //telemetry.addData("originalMagnitude", "(%.2f)", toDegrees(originalMagnitude));


        telemetry.addData("leftFront", "(%.2f)", leftFrontDrive.getPower());
        telemetry.addData("rightFront", "(%.2f)", rightFrontDrive.getPower());
        telemetry.addData("leftRear", "(%.2f)", rightRearDrive.getPower());
        telemetry.addData("rightRear", "(%.2f)", leftRearDrive.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    // when called figures out if it is out of the deadband and returns a double that is designed to spin the opposite of
    public double counterspin() {
        double error = getError(previousHeading); //so we don't have to keep calling this
        if (Math.abs(error) > 5 * deadband) { //if the error is significantly larger than deadband correct more aggressively
            return (error > 0) ? -1 : 1;
        }
        else if (Math.abs(error) >  2 * deadband) { //if the error is slightly larger than deadband be nice
            return (error > 0) ? -0.5 : 0.5;
        }
        else if (Math.abs(error) >  deadband) { //if the error is barely than deadband be nice
            return (error > 0) ? -0.3 : 0.3;
        }
        return 0; //if within deadband chill
    }

    //returns the current heading of the robot relative to the starting position
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    //find difference between where we want to go and where we are
    public double getError(double desiredHeading) {
        return desiredHeading - getHeading();
    }

    public void moveBot(double drive, double rotate, double strafe, double scaleFactor)
    {
        // This module takes inputs, normalizes them to DRIVE_SPEED, and drives the motors
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // How to normalize...Version 3
        //Put the raw wheel speeds into an array
        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = strafe + drive - rotate;
        wheelSpeeds[1] = strafe - drive + rotate;
        wheelSpeeds[2] = strafe - drive - rotate;
        wheelSpeeds[3] = strafe + drive +  rotate;
        // Find the magnitude of the first element in the array
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        // If any of the other wheel speeds are bigger, save that value in maxMagnitude
        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);
            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }
        // Normalize all of the magnitudes to below 1
        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
        // Send the normalized values to the wheels, further scaled by the user

        // Send calculated power to wheels
        leftFrontDrive.setPower(wheelSpeeds[0] * scaleFactor);
        rightFrontDrive.setPower(wheelSpeeds[1] * scaleFactor);
        leftRearDrive.setPower(wheelSpeeds[2] * scaleFactor);
        rightRearDrive.setPower(wheelSpeeds[3] * scaleFactor);
    //return wheelSpeeds;
    }
    public double toDegrees(double radians) { //convert to degrees from radians
        return radians / Math.PI * 180;
    }

    public double toRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private List<Recognition> getRecognitions() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            return updatedRecognitions;

            //uncomment this if all this needs to be printed to console
            /*if (updatedRecognitions != null) {

                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("label (%d)", i), recognition.estimateAngleToObject(AngleUnit.DEGREES));
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();

                 */
        }
        return null;
    }
}