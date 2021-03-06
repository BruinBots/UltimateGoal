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
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.VinceHardwareBruinBot;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


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

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
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

@TeleOp(name="Tyler TeleOp Testing", group="Iterative Opmode")
//@Disabled
public class TylerTeleopTesting extends OpMode
{
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;

    public DcMotorEx ringShooterMotor = null;
    public DcMotorEx wobbleMotor = null;
    public DcMotor intakeMotor = null;

    public Servo fireServo = null;
    public Servo clawServo = null;

    public BNO055IMU imu = null;

    // Variables used to determine robot position on the field and range/bearing to target
    public double               robotX = 0;             // X position in Field Centric Coordinates (in)
    public double               robotY = 0;             // Y position in Field Centric Coordinates (in)
    public double               robotBearing = 0;       // Robot's rotation around the Z axis (CCW is positive) in Field Centric Coordinates (deg)
    public double               targetRange = 0;        // Range from robot's center to target (in)
    public double               targetBearing = 0;      // Heading of the target , relative to the robot's unrotated center (deg)
    public double               relativeBearing = 0;    // Heading to the target from the robot's current bearing. (deg)
    public double               visTgtX = 0;            // Visible Target X component in field centric coordinates
    public double               visTgtY = 0;            // Visible Target Y component in field centric coordinates
    public static double        angleCloseEnough = 2;   // Deadband around angle (deg)
    public static double        rangeCloseEnough = 2;   // Deadband around range (in)
    public static double        shotRange = 80;         // Optimum shot distance behind the shot line (in)
    public double               power = 0.8;            //used to control max drive power
    public boolean              shooterOn = false;      // Track whether the ring shooter is on
    public boolean              intakeFwd = false;       // Track whether the intake is running in the forward direction
    public boolean              intakeRev = false;      // Track whether the intake is running in reverse
    public static double        FIRE_STANDBY_SERVO = 0.77;   // Position for the servo to be in when not firing
    public static double        FIRE_SERVO = 1;         // Position for the servo when firing a ring
    public final double         CLAW_STANDBY_SERVO = 0;
    public final double         CLAW_GRAB = 1;
    public static int           WOBBLE_GRAB = -900;     // Position for grabbing the wobble goal off the field
    public static int           WOBBLE_OVER_WALL = -450; // Position for raising the wobble goal over the wall
    public static int           WOBBLE_CARRY = -630;     // POsition for carrying the wobble goal to the wall
    public double lastwheelSpeeds[] = new double[4];     // Tracks the last power sent to the wheels to assist in ramping power
    public static double        SPEED_INCREMENT = 0.09;  // Increment that wheel speed will be increased/decreased
    public static double        ringVel = 1500;            // Velocity of ring shooter (in ticks, max 1900)

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

    //variables for object detection
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    //private static final String VUFORIA_KEY = "AakkMZL/////AAABmRnl+IbXpU2Bupd2XoDxqmMDav7ioe6D9XSVSpTJy8wS6zCFvTvshk61FxOC8Izf/oEiU7pcan8AoDiUwuGi64oSeKzABuAw+IWx70moCz3hERrENGktt86FUbDzwkHGHYvc/WgfG3FFXUjHi41573XUKj7yXyyalUSoEbUda9bBO1YD6Veli1A4tdkXXCir/ZmwPD9oA4ukFRD351RBbAVRZWU6Mg/YTfRSycyqXDR+M2F/S8Urb93pRa5QjI4iM5oTu2cbvei4Z6K972IxZyiysbIigL/qjmZHouF9fRO4jHoJYzqVpCVYbBVKvVwn3yZRTAHf9Wf77/JG5hJvjzzRGoQ3OHMt/Ch93QbnJ7zN";
    //private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static List<Recognition> lastRecognitions = new ArrayList<Recognition>();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        VinceHardwareBruinBot robot = new VinceHardwareBruinBot();
        // Inititalize last wheel speed
        for (int i = 0; i < lastwheelSpeeds.length; i++){
            lastwheelSpeeds[i] = 0;
        }
        robot.init(hardwareMap);
        {
            //init all drive wheels
            leftFrontDrive = robot.leftFrontDrive;
            rightFrontDrive = robot.rightFrontDrive;
            leftRearDrive = robot.leftRearDrive;
            rightRearDrive = robot.rightRearDrive;

            // init all other motors & servos
            intakeMotor = robot.intakeMotor;
            wobbleMotor = robot.wobbleMotor;
            ringShooterMotor = robot.ringShooterMotor;

            fireServo = robot.fireServo;
            clawServo = robot.clawServo;


            //init imu
            imu = robot.imu;
        }
        initVuforiaNavigation();

        // Reset the wobble motor - Use a repeatable starting position
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*
        // Get PID constants for wobble motor
        int motorIndex = ((robot.wobbleMotor).getPortNumber());
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)robot.wobbleMotor.getController();
        PIDCoefficients pidModified = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION);

        // change coefficients using methods included with DcMotorEx class.
        PIDCoefficients pidNew = new PIDCoefficients(10, 2, 3);
        motorControllerEx.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        */
        //wobbleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidNew);
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
        fireServo.setPosition(FIRE_STANDBY_SERVO);
        clawServo.setPosition(CLAW_STANDBY_SERVO);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        double correctedAngle = Math.toDegrees(Math.atan2(drive, strafe)) + 90;
        double inputMagnitude = Math.hypot(drive, strafe);
        double correctedDrive = Math.sin(Math.toRadians(correctedAngle)) * inputMagnitude;
        double correctedStrafe = Math.cos(Math.toRadians(correctedAngle)) * inputMagnitude;

        findRobotPosition();  // Call Vuforia routines, get robot position

        // Intake  Operator Actions
        if (gamepad1.a || gamepad1.b || gamepad1.x){  // Operator wants to control the intake
            if (gamepad1.a){    // A turns the intake on for bringing in rings
                intakeMotor.setPower(1);
            }
            else if (gamepad1.b) {  // B reverses the intake
                intakeMotor.setPower(-1);

            } else if (gamepad1.x){    // X stops the intake
                intakeMotor.setPower(0);
            }
        }

        // Ring Shooter Operator Actions
        /*if (gamepad1.dpad_up || gamepad1.dpad_down){
            if (gamepad1.dpad_up) {
                if (ringVel <= 170000) {
                    ringVel = ringVel + 10000;
                }
            } else if (gamepad1.dpad_down){
                    if (ringVel >= 10000){
                        ringVel = ringVel - 10000;
                    } else{
                        ringVel = 0;
                    }
                }
            }
        */

        if (gamepad1.right_bumper){
            fireServo.setPosition(FIRE_SERVO);
        } else{
            fireServo.setPosition(FIRE_STANDBY_SERVO);
        }
        if (gamepad1.left_trigger > 0.5){
            ringShooterMotor.setVelocity(ringVel);
            telemetry.addData("shooterVelocity", ringShooterMotor.getVelocity());
        }else {
            ringShooterMotor.setPower(0);
        }

        // Wobble Goal acions here
        if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.left_bumper || gamepad1.left_stick_button || gamepad1.right_stick_button){
            if(gamepad1.dpad_left){
                wobbleMotor.setTargetPosition(WOBBLE_GRAB);
                wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleMotor.setPower(.4);
            } else if (gamepad1.dpad_right){
                wobbleMotor.setTargetPosition(WOBBLE_CARRY);
                wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleMotor.setPower(.4);
            } else if (gamepad1.left_bumper) {
                wobbleMotor.setTargetPosition(WOBBLE_OVER_WALL);
                wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleMotor.setPower(.4);
            }
            else if (gamepad1.left_stick_button) {
                clawServo.setPosition(CLAW_GRAB);
            }
            else if (gamepad1.right_stick_button)
                clawServo.setPosition(CLAW_STANDBY_SERVO);
        }
        // Insert operator driving actions here
        if ((targetVisible) && (gamepad1.y)) {
            //  Allow the operator to position the robot to shoot a ring if a target is visible
            alignRobotToShoot();
        }
        else {
            moveBot(drive, rotate, strafe, power); //Basic Robot-centric Frame Driving
        }

        // Telemetry Section
        //telemetry.addData("Ring Velocity", ringVel);
        //telemetry.addData("Actual Velocity", ringShooterMotor.getVelocity());
        telemetry.addData("Wobble Goal Encoder", wobbleMotor.getCurrentPosition());
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

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();

        //closes object detection to save system resouces
        if (tfod != null) {
            tfod.shutdown();
        }
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

        // Positive Drive moves forward
        // Positive Rotation spins Left
        // Positive Strafe moves right

        //Put the raw wheel speeds into an array
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = drive - rotate - strafe; // Right Read
        wheelSpeeds[1] = drive - rotate + strafe; // Right Front
        wheelSpeeds[2] = drive + rotate + strafe; // Left Rear
        wheelSpeeds[3] = drive + rotate - strafe; // Left Front
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

        // Compare last wheel speeds to commanded wheel speeds and ramp as necessary
        for (int i = 0; i < lastwheelSpeeds.length; i++){
            // If the commanded speed value is more than SPEED_INCREMENT away from the last known wheel speed
            if (Math.abs(wheelSpeeds[i] - lastwheelSpeeds[i]) > SPEED_INCREMENT){
                // Set the current wheel speed to the last wheel speed plus speed increment in the signed directin of the difference
                wheelSpeeds[i] = lastwheelSpeeds[i] + Math.copySign(SPEED_INCREMENT,wheelSpeeds[i] - lastwheelSpeeds[i]);
            }
        }

        // Send the normalized values to the wheels, further scaled by the user
        rightRearDrive.setPower(wheelSpeeds[0] * scaleFactor);
        rightFrontDrive.setPower(wheelSpeeds[1] * scaleFactor);
        leftRearDrive.setPower(wheelSpeeds[2] * scaleFactor);
        leftFrontDrive.setPower(wheelSpeeds[3] * scaleFactor);

        // Save the last wheel speeds to assist in ramping
        for (int i = 0; i < lastwheelSpeeds.length; i++){
            lastwheelSpeeds[i] = wheelSpeeds[i];
        }
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
        final float CAMERA_FORWARD_DISPLACEMENT  = 7.0f * mmPerInch;   // eg: Camera is 7 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = -4.0f * mmPerInch;     // eg: Camera is 4 inches to the RIGHT the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsUltimateGoal.activate();
    }


    // Aligns the robot with the goal and moves to an acceptable shooting range
    public void alignRobotToShoot(){
        double rangeError;
        double angleError;
        double rangePercent = .1; // Percent of range error to pass to moveBot
        double anglePercent = .5; // Percent of angle error to pass to moveBot

        boolean rangeGood = false;
        boolean angleGood = false;

        if (Math.abs(targetRange - shotRange)> rangeCloseEnough){
            // Still not in optimal shot position
            rangeError = targetRange - shotRange;
        } else {
            rangeGood = true;
            rangeError = 0;
        }
        if (Math.abs(relativeBearing) > angleCloseEnough){
            // still not pointing the target
            angleError = relativeBearing;
        }  else{
            angleGood = true;
            angleError = 0;
        }

        if (!rangeGood || !angleGood){
            moveBot(rangePercent * rangeError, anglePercent * angleError, 0, 0.2);
        }
    }

    public void findRobotPosition(){
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // Get the position of the visible target and put the X & Y position in visTgtX and visTgtY
                // We will use these for determining bearing and range to the visible target
                OpenGLMatrix tgtLocation = trackable.getLocation();
                VectorF tgtTranslation = tgtLocation.getTranslation();
                visTgtX = tgtTranslation.get(0) / mmPerInch;
                visTgtY = tgtTranslation.get(1) / mmPerInch;
                //telemetry.addData("Tgt X & Y", "{X, Y} = %.1f, %.1f", visTgtX, visTgtY);

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
            //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            // Robot position is defined by the standard Matrix translation (x and y)
            robotX = translation.get(0)/ mmPerInch;
            robotY = translation.get(1)/ mmPerInch;

            // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
            robotBearing = rotation.thirdAngle;

            // target range is based on distance from robot position to the visible target
            // Pythagorean Theorum
            targetRange = Math.sqrt((Math.pow(visTgtX - robotX, 2) + Math.pow(visTgtY - robotY, 2)));

            // target bearing is based on angle formed between the X axis to the target range line
            // Always use "Head minus Tail" when working with vectors
            targetBearing = Math.toDegrees(Math.atan((visTgtY - robotY)/(visTgtX - robotX)));

            // Target relative bearing is the target Heading relative to the direction the robot is pointing.
            // This can be used as an error signal to have the robot point the target
            relativeBearing = targetBearing - robotBearing;
            // Display the current visible target name, robot info, target info, and required robot action.

            telemetry.addData("Robot", "[X]:[Y] (Heading) [%5.0fin]:[%5.0fin] (%4.0f°)",
                    robotX, robotY, robotBearing);
            telemetry.addData("Target", "[TgtRange] (TgtBearing):(RelB) [%5.0fin] (%4.0f°):(%4.0f°)",
                    targetRange, targetBearing, relativeBearing);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
    }
}