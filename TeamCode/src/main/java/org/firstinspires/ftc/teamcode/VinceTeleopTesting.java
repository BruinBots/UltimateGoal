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

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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

@TeleOp(name="Vince TeleOp Testing", group="Iterative Opmode")
//@Disabled
public class VinceTeleopTesting extends OpMode
{
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public BNO055IMU imu = null;

    double power = 0.7; //used to control max drive power

    private static final String VUFORIA_KEY = "AakkMZL/////AAABmRnl+IbXpU2Bupd2XoDxqmMDav7ioe6D9XSVSpTJy8wS6zCFvTvshk61FxOC8Izf/oEiU7pcan8AoDiUwuGi64oSeKzABuAw+IWx70moCz3hERrENGktt86FUbDzwkHGHYvc/WgfG3FFXUjHi41573XUKj7yXyyalUSoEbUda9bBO1YD6Veli1A4tdkXXCir/ZmwPD9oA4ukFRD351RBbAVRZWU6Mg/YTfRSycyqXDR+M2F/S8Urb93pRa5QjI4iM5oTu2cbvei4Z6K972IxZyiysbIigL/qjmZHouF9fRO4jHoJYzqVpCVYbBVKvVwn3yZRTAHf9Wf77/JG5hJvjzzRGoQ3OHMt/Ch93QbnJ7zN";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    // the height of the center of the target image above the floor

    VuforiaTrackables targetsUltimateGoal = null;
    List<VuforiaTrackable> allTrackables = null;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //variables for object detection, not navigation
    /*
    //variables for object detection
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AakkMZL/////AAABmRnl+IbXpU2Bupd2XoDxqmMDav7ioe6D9XSVSpTJy8wS6zCFvTvshk61FxOC8Izf/oEiU7pcan8AoDiUwuGi64oSeKzABuAw+IWx70moCz3hERrENGktt86FUbDzwkHGHYvc/WgfG3FFXUjHi41573XUKj7yXyyalUSoEbUda9bBO1YD6Veli1A4tdkXXCir/ZmwPD9oA4ukFRD351RBbAVRZWU6Mg/YTfRSycyqXDR+M2F/S8Urb93pRa5QjI4iM5oTu2cbvei4Z6K972IxZyiysbIigL/qjmZHouF9fRO4jHoJYzqVpCVYbBVKvVwn3yZRTAHf9Wf77/JG5hJvjzzRGoQ3OHMt/Ch93QbnJ7zN";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static List<Recognition> lastRecognitions = new ArrayList<Recognition>();
*/

    //variables to maintain a heading
    public double previousHeading = 0;
    public double deadband = toRadians(3);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        HardwareBruinBot robot = new HardwareBruinBot();

        robot.init(hardwareMap);

        //init all drive wheels
        leftFrontDrive = robot.leftFrontDrive;
        rightFrontDrive = robot.rightFrontDrive;
        leftRearDrive = robot.leftRearDrive;
        rightRearDrive = robot.rightRearDrive;

        //init imu
        imu = robot.imu;

        //initVuforiaNavigation();


        //initialize vision components
        /*initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }*/

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
        //targetsUltimateGoal.activate();
        //runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        double x  = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        //double correctedAngle = getError(Math.atan2(y, x));

        //adjust rotation parameter to spin opposite to rotation drift

        /*if (r != 0)
            previousHeading = getHeading(); //makes sure that while intentionally spinning no correction is being made
        else
            r += counterspin();*/

        //double originalMagnitude = Math.hypot(y, x); //how far the joystick is being pressed
        //double correctedX = Math.cos(correctedAngle) * originalMagnitude; //break it back up to send to movebot
        //double correctedY = Math.sin(correctedAngle) * originalMagnitude;

        //double[] wheelSpeeds = moveBot(x, r, y, power); //to control from a robot perspective
        moveBot(x, r, y, power); //Basic Robot-centric Frame Driving
        // check all the trackable targets to see which one (if any) is visible.
        /*targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }*/
        telemetry.update();

        //get all recognitions from object detection code and displays an ArrayList with all recognitions
        /*
        List<Recognition> recognitions = getRecognitions();
        if (recognitions != null)
            lastRecognitions = recognitions;
        telemetry.addData("detectedRecognitions", lastRecognitions);
*/
        /*
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
        telemetry.update();*/
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

        // Disable Tracking when we are done;
        //targetsUltimateGoal.deactivate();

        //closes object detection to save system resouces
        /*if (tfod != null) {
            tfod.shutdown();
        }*/
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
        else if (Math.abs(error) >  deadband) { //if the error is slightly larger than deadband be nice
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

        //adjust rotation parameter to spin opposite to rotation drift
        /*if (rotate != 0)
            previousHeading = getHeading(); //makes sure that while intentionally spinning no correction is being made
        else
            rotate += counterspin();*/

        wheelSpeeds[0] = strafe + drive - rotate;
        wheelSpeeds[1] = strafe - drive + rotate;// VInce: I think 1 & 2 are swapped which is why we have wonky driving
        wheelSpeeds[2] = strafe - drive - rotate;// should be -- and -+ respectively
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

    public void initVuforiaNavigation() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

    }

    public double toDegrees(double radians) { //convert to degrees from radians
        return radians / Math.PI * 180;
    }

    public double toRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    //used for object detection
    /**
     * Initialize the TensorFlow Object Detection engine.
     *
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
     if (updatedRecognitions != null) {
     return updatedRecognitions;

     /*telemetry.addData("# Object Detected", updatedRecognitions.size());

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

     *
     }
     }
     return null;
     }

     private void stupidGetRecognitions() {
     if (tfod != null) {
     // getUpdatedRecognitions() will return null if no new information is available since
     // the last time that call was made.
     List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
     if (updatedRecognitions != null) {
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
     }
     }
     }

     private List<Recognition> getSingles() {
     List<Recognition> singles = new ArrayList();
     List<Recognition> recognition = getRecognitions();
     for (int i = 0; i < recognition.size(); i++) {
     if (recognition.get(i).getLabel().equals("Single"));
     singles.add(recognition.get(i));
     }

     return singles;
     }
     */
}