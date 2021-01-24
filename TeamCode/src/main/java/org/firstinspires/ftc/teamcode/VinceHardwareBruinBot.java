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

//all the imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//skystone hardwaremap

//the place where all the variables are defined
public class VinceHardwareBruinBot {
    /* Public OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;

    public BNO055IMU imu = null;

    public DcMotorEx ringShooterMotor = null;
    public DcMotorEx wobbleMotor = null;
    public DcMotor intakeMotor = null;

    public Servo fireServo = null;



    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public VinceHardwareBruinBot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftRearDrive = hwMap.get(DcMotor.class, "leftRearDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");

        ringShooterMotor = hwMap.get(DcMotorEx.class, "ringShooterMotor");
        wobbleMotor = hwMap.get(DcMotorEx.class, "wobbleMotor");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        //Initialize Servos
        fireServo = hwMap.get(Servo.class, "fireServo");

        // REV IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //set drive motor directions
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);

        ringShooterMotor.setDirection(DcMotor.Direction.REVERSE); // Makes 1 the shooting direction
        wobbleMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE); // Makes 1 drive the intake to suck in rings

        // Set drive motors to zero power
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Set other motors to zero power
        ringShooterMotor.setPower(0);
        wobbleMotor.setPower(0);
        intakeMotor.setPower(0);

        // Set wobble goal motor brake behavior to hold the goal
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set intake so it doesn't stop too quickly
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Ring Shooter so it doesn't stop too quickly
        ringShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ringShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set drive motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
}