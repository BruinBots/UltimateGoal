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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class TeleOpDriving extends OpMode
{
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public BNO055IMU gyro = null;

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
        /*leftFrontDrive  = robot.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = robot.get(DcMotor.class, "right_front_drive");
        leftRearDrive = robot.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = robot.get(DcMotor.class, "right_rear_drive");
*/

        leftFrontDrive = robot.leftFrontDrive;
        rightFrontDrive = robot.rightFrontDrive;
        leftRearDrive = robot.leftRearDrive;
        rightRearDrive = robot.rightRearDrive;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /*leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

*/


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.  <--- I don't think this is necessary because of iterative op mode. so just dont press play for 25 ms
        //while (!gyro.isGyroCalibrated())
        //{
            
        //}

        telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", gyro.getCalibrationStatus().toString());
        telemetry.update();


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
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;

        double power = 0.3;
        double x  = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        double[] wheelSpeeds = moveBot(x, r, y, power);
/*
        //beginning of big brain -------------------------------------------------------------------.

        //!!!!!!!!!!!!!!!!!!!!!!!!!!REPLACE WITH INFO FROM GYRO SENSOR TO CORRECT OFFSET TO ROBOT ALWAYS GOES IN DIRECTION OF STICK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        double angleFromGyro = 0;//gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;//1.0; also probably wrong one but just change the angle order
        double desiredAngle = (x < 0)? Math.atan(y / x) + Math.PI : Math.atan(y / x);
        if (desiredAngle < 0)
            desiredAngle += 2 * Math.PI;
        double correctedAngle = desiredAngle - angleFromGyro;


        //correct y
        double newY = Math.sin(correctedAngle);
        //correct x
        double newX = Math.cos(correctedAngle);

        //back to smooth brain ---------------------------------------------------------------------



        /*double drive = newY;
        double strafe = newX;
        double rotate = r;*/

        /*

        double ro = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = -gamepad1.left_stick_y + gamepad1.left_stick_x;//ro * Math.cos(robotAngle) + rightX;
        double v2 = -gamepad1.left_stick_y - gamepad1.left_stick_x;//ro * Math.sin(robotAngle) - rightX;
        double v3 = -gamepad1.left_stick_y - gamepad1.left_stick_x;//ro * Math.sin(robotAngle) + rightX;
        double v4 = -gamepad1.left_stick_y + gamepad1.left_stick_x;//ro * Math.cos(robotAngle) - rightX;

        leftFrontPower = v1;//Range.clip(drive + strafe + rotate, -1.0, 1.0);
        leftRearPower = v2;//Range.clip(drive - strafe + rotate, -1.0, 1.0);
        rightFrontPower = v3;//.clip(drive - strafe - rotate, -1.0, 1.0);
        rightRearPower = v4;//Range.clip(drive + strafe - rotate, -1.0, 1.0);
*/

/*


        double max = Math.max(Math.max(leftFrontPower, rightFrontPower), Math.max(leftRearPower, rightRearPower));
        leftFrontPower /= max;
        rightFrontPower /= max;
        leftRearPower /= max;
        rightRearPower /= max;
*/
        leftFrontPower = wheelSpeeds[0];
        rightFrontPower = wheelSpeeds[1];
        leftRearPower = wheelSpeeds[2];
        rightRearPower = wheelSpeeds[3];

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("GyroAngle", "(%.2f) degrees", angleFromGyro);
        //telemetry.addData("Angle", "(%.2f)", robotAngle);
        telemetry.addData("controllerX", "(%.2f)", gamepad1.left_stick_x);
        telemetry.addData("controllerY", "(%.2f)", -gamepad1.left_stick_y);
        telemetry.addData("leftFront", "(%.2f)", leftFrontPower);
        telemetry.addData("rightFront", "(%.2f)", rightFrontPower);
        telemetry.addData("leftRear", "(%.2f)", rightRearPower);
        telemetry.addData("rightRear", "(%.2f)", leftRearPower);
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
    }

    public double[] moveBot(double drive, double rotate, double strafe, double scaleFactor)
    {
        // This module takes inputs, normalizes them to DRIVE_SPEED, and drives the motors
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // How to normalize...Version 3
        //Put the raw wheel speeds into an array
        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = strafe + drive - rotate;
        wheelSpeeds[1] = strafe - drive - rotate;
        wheelSpeeds[2] = strafe - drive + rotate;
        wheelSpeeds[3] = strafe + drive + rotate;
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
    return wheelSpeeds;
    }

}