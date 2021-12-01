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
//Add speaker, spoiler, mudflaps (and reverse alarm), tinted Glass
package Spudnik;

import android.app.admin.DelegatedAdminReceiver;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@Autonomous(name="SpudnikAnonRed2 (click this one)", group="Linear Opmode") //auto mode
//@Disabled
public class SpudnikAnonRed extends LinearOpMode { //class config

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime(); //time
    private DcMotor leftDrive = null; //left motor
    private DcMotor rightDrive = null; //right motor

    private DcMotor duck = null; //duck spinner for carousel
    private DcMotor armRaise = null; //arm raiser
    private Servo clampServo = null; //servo for clamp
    private Servo spoiler = null;
    private RevTouchSensor button = null;

    private int stage = 0; //stage for auto

    private DistanceSensor spinSensor = null;

    private BNO055IMU imu = null; //imu declaration

    private Orientation angles; //heading degree variable
    private double curHeading; //numerical heading in double form

    private Acceleration gravity; //acceleration
    private double accX; //numerical acceleration x
    private double accY; //numerical acceleration y

    private Acceleration overall; //overall acceleration
    private double overX; //numerical acceleration overall x
    private double overY; //numerical acceleration overall y

    private Position map; //position of robot on map
    private double mapX; //numerical map position x
    private double mapY; //numerical map position y

    private double leftPower= 0; //declare motor power on the left
    private double rightPower = 0; //declare motor power on the right
    private double spinPower = 0; //power for duck spin
    private double raisePower = 0; //declare power for arm pwo
    private double numSen = 0;



    @Override
    public void runOpMode() {
        initEverything();

        while (opModeIsActive()) { //while auto is running
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //reset calibration
            checkOrientation(); //check the orientation
            checkAcceleration(); //check acceleration
            checkOverallAcceleration(); //check overall acceleration
            checkNavigation(); //check navigation
            // Setup a variable for each drive wheel to save power level for telemetry

            leftPower= 0; //declare motor power on the left
            rightPower = 0; //declare motor power on the right
            spinPower = 0; //power for duck spin
            raisePower = 0; //declare power for arm pwo
            numSen = spinSensor.getDistance(DistanceUnit.CM);



           if(stage == 0){
                leftDrive.setPower(-10);
                rightDrive.setPower(-10);
                duck.setPower(100);
                sleep(300);
                stage = 1;
           }
           if(stage == 1){
               leftDrive.setPower(0);
               rightDrive.setPower(0);
               duck.setPower(100);
               sleep(4000);
               stage = 2;
           }
           if(stage == 2){
               leftDrive.setPower(10);
               if(curHeading <= -45) stage = 3;
           }
           if(stage == 3){
               leftDrive.setPower(100);
               rightDrive.setPower(100);
               armRaise.setPower(100);
               duck.setPower(0);
               sleep(1000);
               stage = 4;
           }
           if(stage == 4){
               armRaise.setPower(0);
               leftDrive.setPower(100);
               rightDrive.setPower(100);
               sleep(4000);
               stage = 5;
           }
           if(stage == 5){
               leftDrive.setPower(0);
               rightDrive.setPower(0);
           }






            telemetry.addData("Degrees", "* (%.2f)", curHeading); //degrees telemetry
            telemetry.addData("X - Y Acceleration", "X (%.2f), Y (%2f)", accX, accY); //acceleration telemetry
            telemetry.addData("X - Y Overall", "X (%.2f), Y (%2f)", overX, overY); //overall acceleration telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString()); //run time telemetry
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // wheel telemetry
            telemetry.addData("Clamp", "Trigger1 (%.2f), Trigger2 (%.2f)", gamepad1.right_trigger, gamepad2.right_trigger); //servo clamp telemetry
            telemetry.addData("DuckSpin", "Motor (%2f)", spinPower); //duck spinner motor telemetry
            telemetry.addData("ArmRaise", "Motor (%2f)", raisePower); //arm motor telemetry
            telemetry.addData("stage", stage);
            telemetry.update(); //update






        }

    }
    public void getNumbers(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //reset calibration
        checkOrientation(); //check the orientation
        checkAcceleration(); //check acceleration
        checkOverallAcceleration(); //check overall acceleration
        checkNavigation(); //check navigation
        leftPower= 0; //declare motor power on the left
        rightPower = 0; //declare motor power on the right
        spinPower = 0; //power for duck spin
        raisePower = 0; //declare power for arm pwo
        numSen = spinSensor.getDistance(DistanceUnit.CM);
    }
    public void initEverything(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive"); //motor hardware class
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive"); //motor hardware class
        duck = hardwareMap.get(DcMotor.class, "spin"); //motor hardware class
        clampServo = hardwareMap.get(Servo.class, "clamp"); //servo hardware class
        armRaise = hardwareMap.get(DcMotor.class, "raise"); //servo hardware class
        spoiler = hardwareMap.get(Servo.class, "spoiler");  //servo hardware class
        spinSensor = hardwareMap.get(DistanceSensor.class, "spinSensor");
        button = hardwareMap.get(RevTouchSensor.class, "button");

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //imu hardware class
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //make new parameters
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //degree is the unit
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //in meters per second
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true; //logInput
        parameters.loggingTag          = "IMU"; //logs as IMU (see logcat)
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //log acceleration
        imu.initialize(parameters); //initialize all parameters
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //calibrate the parameters



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.REVERSE); //left is reversed because it is flipped
        rightDrive.setDirection(DcMotor.Direction.FORWARD); //set direction
        duck.setDirection(DcMotor.Direction.FORWARD); //set direction
        armRaise.setDirection(DcMotorSimple.Direction.FORWARD); //set direction of arm raise




        // Wait for the game to start (driver presses PLAY)
        waitForStart(); //wait for start
        runtime.reset(); //once started, reset clock

        // run until the end of the match (driver presses STOP)
    }
    public void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        curHeading = angles.firstAngle;
    }
    public void checkAcceleration(){
        gravity = this.imu.getAcceleration();
        accX = gravity.xAccel;
        accY = gravity.yAccel;
    }
    public void checkOverallAcceleration(){
        overall = this.imu.getOverallAcceleration();
        overX = overall.xAccel;
        overY = overall.yAccel;
    }
    public void checkNavigation(){
        map = this.imu.getPosition();
        mapX = map.x;
        mapY = map.y;
        telemetry.addData("X - Y Map", "X (%2f), Y (%2f)", mapX, mapY);
    }

}
