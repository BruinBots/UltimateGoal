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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//skystone hardwaremap

//the place where all the variables are defined
public class HardwareBruinBot
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor rightRearDrive = null;
//    public DcMotor armExtendMotor = null;
//    public DcMotor armLiftMotor = null;
//    public DcMotor tapeMotor = null;

//    public Servo clawServo = null;
//    public Servo leftPlatformServo;
//    public Servo rightPlatformServo;
//    public Servo capstoneServo = null;

//    public DigitalChannel frontTouchSensor;
//    public DigitalChannel backTouchSensor;

    public BNO055IMU gyro;

//    public ModernRoboticsI2cRangeSensor rangeSensor;
//    public ColorSensor colorSensor;
//    public AnalogInput sonarSensor;
//    public DistanceSensor backDistance;

//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;
//    public static final double ARM_EXT_SERVO   =  0.5 ; //this must be changed later

    /* local OpMode members. */
    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBruinBot(){
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

//        armExtendMotor = hwMap.get(DcMotor.class, "armExtendMotor");
//        armLiftMotor = hwMap.get(DcMotor.class, "armLiftMotor");
//        tapeMotor = hwMap.get(DcMotor.class, "tapeMotor");

        //Initialize Servos
//        clawServo = hwMap.get(Servo.class, "clawServo");
//        leftPlatformServo = hwMap.get(Servo.class, "leftPlatformServo");
//        rightPlatformServo = hwMap.get(Servo.class, "rightPlatformServo");
//        capstoneServo = hwMap.get(Servo.class, "capstoneServo");

        // Touch Sensors
//        frontTouchSensor = hwMap.get(DigitalChannel.class, "frontTouchSensor");
//        backTouchSensor = hwMap.get(DigitalChannel.class, "backTouchSensor");
//        frontTouchSensor.setMode(DigitalChannel.Mode.INPUT);
//        backTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        //color sensor
//        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        // REV IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        gyro = hwMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        //Initialize I2C Sensors
//        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
//        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //Initialize Analog Sonar Sensor
//        sonarSensor = hwMap.get(AnalogInput.class,"sonarSensor");

//        backDistance = hwMap.get(DistanceSensor.class, "sensor_range");

        //set drive motor directions
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set drive motors to zero power
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Set drive motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // make lifting motor brake when not in use
//        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}