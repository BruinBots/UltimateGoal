package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Vince Linear Autonomous", group="Linear Opmode")
//@Disabled
public class VinceAutonomous extends LinearOpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftRearDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightRearDrive = null;

    public DcMotorEx ringShooterMotor = null;
    public DcMotorEx wobbleMotor = null;
    public DcMotor intakeMotor = null;

    public Servo fireServo = null;

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
    public double               power = 0.7;            //used to control max drive power
    public boolean              shooterOn = false;      // Track whether the ring shooter is on
    public boolean              intakeFwd = false;       // Track whether the intake is running in the forward direction
    public boolean              intakeRev = false;      // Track whether the intake is running in reverse
    public static double        STANDBY_SERVO = 0.77;   // Position for the servo to be in when not firing
    public static double        FIRE_SERVO = 1;         // Position for the servo when firing a ring
    public static int           WOBBLE_GRAB = -900;     // Position for grabbing the wobble goal off the field
    public static int           WOBBLE_OVER_WALL = -550; // Position for raising the wobble goal over the wall
    public static int           WOBBLE_CARRY = -630;     // POsition for carrying the wobble goal to the wall
    public double lastwheelSpeeds[] = new double[4];     // Tracks the last power sent to the wheels to assist in ramping power
    public static double        SPEED_INCREMENT = 0.09;  // Increment that wheel speed will be increased/decreased
    public static double        ringVel = 1500;            // Velocity of ring shooter (in ticks, max 1900)

    private boolean             found = false;                        // indicates when a wobble goal has been found
    private String              wobbleBox = "1stBox";                  // Indicates what box to drive to to drop off the wobblegoal

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

    //VinceHardwareBruinBot robot = new VinceHardwareBruinBot();



    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        VinceHardwareBruinBot robot = new VinceHardwareBruinBot();
        // Inititalize last wheel speed
        for (int i = 0; i < lastwheelSpeeds.length; i++) {
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

            //init imu
            imu = robot.imu;
        }

        // Reset the wobble motor - Use a repeatable starting position
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //initialize vision components
        initVuforia();
        initTfod();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        //initVuforiaNavigation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        tfod.activate();
        runtime.reset();
        while (!isStarted()) {
            // Put things to do prior to start in here
            // Detect the ring stack
            telemetry.addData("WobbleBox: ", wobbleBox);
            telemetry.update();
            Recognition detection = getStrongestRecognition(); //gets the recognition with the highest confidence

            if (detection != null) {
                if (detection.getLabel().equals("Single")) {
                    wobbleBox = "2ndBox";
                    found = true;
                } else if (detection.getLabel().equals("Quad")) {
                    wobbleBox = "3rdBox";
                    found = true;
                }

            }
        }


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tfod.shutdown(); //deinitialize object detection

            initNavigation(); //initialize and activate navigation
            targetsUltimateGoal.activate();

            switch (wobbleBox){
                case "1stBox":{ // Drop Zone A
                    // Drive forward 8 ft, turn right 45 degrees, and drop the goal
                    driveByGyroEncoder(.5,0,8*12);
                    // Turn right to 45 degrees by gyro
                    gyroSpin(-45);
                    // spit out the wobble goal
                    intakeMotor.setPower(.1);
                    sleep(2000);
                    intakeMotor.setPower(0);
                    // Turn back to zero degrees by gyro
                    gyroSpin(0);
                    // Strafe Right for a bit
                    //moveBot(0,getError(0),.5,.5);
                    //sleep(2000);
                    stopBot();
                    // Drive forward until we see the goal
                    while (!findRobotPosition() && !isStopRequested()){
                        moveBot(.5,0,0,.5);
                    }
                    stopBot();
                    // Align to shoot
                    while (!alignRobotToShoot() && !isStopRequested()){
                        moveBot(-.5,0,0,.5);
                    }
                    stopBot();
                    // Shoot 3 rings
                    shoot3Rings();
                    // Get over the white line by driving forward until we're on the positive side of the ield

                    while (robotX < 0 && !isStopRequested()){
                        moveBot(.1,0,0,.5);
                    }
                    stopBot();
                }
                case "2ndBox":{// Drop Zone B

                }
                case "3rdBox":{ // Drop Zone C

                }
            }
            // Navigate to the wobble goal drop box


            // Drop the wobble goal


            // Find the tower target

            // Autohome in a safe direction to shoot

            // Shoot 3 rings


            // Park over the white line


        }
    }
    private void gyroSpin(double heading) {
        // This function spins the robot in place to a desired heading

        // Get the current heading error between actual and desired
        double error = getError(heading);
        // While we are greater than 5 degrees from desired heading (5 seems to work best)
        while (!isStopRequested() && Math.abs(error) > 5) {
            // Rotate the robot in the correct direction.
            // Don't use more than 0.3 input power or it goes too fast
            if (error < 0 && Math.abs(error) > 5) {
                moveBot(0, -0.3, 0, 0.4);
            } else {
                moveBot(0, 0.3, 0, 0.4);
            }
            //Check the error again for the next loop
            error = getError(heading);
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
        stopBot();
    }
    private void stopBot(){
        moveBot(0,0,0,0);
    }
    private void shoot3Rings(){
        ringShooterMotor.setVelocity(1500);
        sleep(1000);
        for (int k = 0; k < 3; k++){
            fireServo.setPosition(FIRE_SERVO);
            sleep(500);
            fireServo.setPosition(STANDBY_SERVO);
            sleep(500);
        }

    }
// This function drives on a gyro angle (deg) for a defined distance (in)
    private void driveByGyroEncoder(double speed,double gyroAngle, double distance){
        double ticks = 0;
        int ppr = 134;  // Pulses per revolution
        double distPerRev = 3.14*4;
        ticks = leftFrontDrive.getCurrentPosition() + (distance/distPerRev) * ppr;

        while (leftFrontDrive.getCurrentPosition() < ticks && !isStopRequested()) {
            moveBot(speed, .1 * getError(0), 0, .5);
            telemetry.addData("Ticks", leftFrontDrive.getCurrentPosition());
            telemetry.update();
        }

    }
    private Recognition getStrongestRecognition() {
        updateRecognitions();

        if (lastRecognitions == null || lastRecognitions.size() == 0) //dont panic, will short circuit if the first condition is true
            return null;

        Recognition highestConfidence = lastRecognitions.get(0);

        //for (int i = 0; i < lastRecognitions.size(); i++) {
        //    if (highestConfidence.getConfidence() < lastRecognitions.get(i).getConfidence())
        //        highestConfidence = lastRecognitions.get(i);
        //}

        //above is not necessary as the call to AnnotatedYuvRgbFrame.getRecognitions returns a sorted list of recognitions by confidence

        return highestConfidence;
    }

    private void updateRecognitions() {
        if (tfod == null) //check so we don't throw error
            return;

        List<Recognition> updated = tfod.getUpdatedRecognitions(); //will pull list of recognitions from model
        //will be null if there is no new detections
        //objects referenced from the last list returned will continue to be tracked which is why we have to update the list only when

        if (updated != null) {
            lastRecognitions = updated;
        }
    }

    // activate object detection model
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId); //comment to not use camera monitor

        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    // ONLY USE THIS ONCE OTHERWISE PANIC WILL OCCUR
    private void initVuforia() {
        //Vuforia.deinit(); //uncomment to use object detection after navigation


        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = new ClosableVuforiaLocalizer(parameters);
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
    public boolean alignRobotToShoot(){
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

        return (rangeGood && angleGood);
    }

    public boolean findRobotPosition(){
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
        return targetVisible;
    }

    private void initNavigation() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        //Vuforia.deinit(); //comment if navigation is used first

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        // Make sure extended tracking is disabled for this example.
        //parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        //vuforia = new ClosableVuforiaLocalizer(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        System.out.println(vuforia.toString());
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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
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
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

    }

}
