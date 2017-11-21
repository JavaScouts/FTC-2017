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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LZRobot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="FinalBlue2")

public class FinalBlue2 extends LinearOpMode {

    /* Declare OpMode members. */
    LZRobot robot = new LZRobot();  // Use a Pushbot's hardware
    LZNavSys nav = new LZNavSys();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    private double input;
    private double prevRangeValue = 0.0;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);
        nav.initVuforia(this, robot);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.backLDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.backRDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        nav.activateTracking();
        telemetry.addData("ColorSensorRed", robot.color.red());
        telemetry.addData("ColorSensorBlue", robot.color.blue());
        telemetry.addData("Range", robot.range1.getDistance(DistanceUnit.CM));

        telemetry.addData("Relic:", nav.whatRelic());

        input = robot.range1.getDistance(DistanceUnit.CM);
        if (input > 100.0 && prevRangeValue <= 100.0)
        {
            input = prevRangeValue;
        }
        else
        {
            prevRangeValue = input;
        }
        telemetry.update();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
       // robot.s1.setPosition(1.0);
        robot.s2.setPosition(1.0);
        encoderDrive(DRIVE_SPEED, 0, 0, 0, 0, 7, 5);
        if(robot.color.blue() > 0){
            encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 0, 5);
            robot.s2.setPosition(0);
            encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 0, 5);
        } else {
            encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 0, 5);
            robot.s2.setPosition(0);
            encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 0, 5);
        }
        if (nav.whatRelic() == "LEFT"){
            encoderDrive(DRIVE_SPEED, 15, 15, 15, 15, 0, 5);
            encoderDrive(DRIVE_SPEED, -10, 10, 10, -10, 0, 5);
            encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 0, 5);
        } else if (nav.whatRelic() == "CENTER"){
            encoderDrive(DRIVE_SPEED, 15, 15, 15, 15, 0, 5);
            encoderDrive(DRIVE_SPEED, -15, 15, 15, -15, 0, 5);
            encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 0, 5);

        } else {
            encoderDrive(DRIVE_SPEED, 15, 15, 15, 15, 0, 5);
            encoderDrive(DRIVE_SPEED, -20, 20, 20, -20, 0, 5);
            encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 0, 5);

        }
        encoderDrive(0, 0, 0, 0, 0, 0, 1);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double  backLInches, double rightInches, double backRInches, double armInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newBackLTarget;
        int newBackRTarget;
        int newArmTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackLTarget = robot.backLDrive.getCurrentPosition() + (int)(backLInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackRTarget = robot.backRDrive.getCurrentPosition() + (int)(backRInches * COUNTS_PER_INCH);
            newArmTarget = robot.slide.getCurrentPosition() + (int)(armInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.backLDrive.setTargetPosition(newBackLTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.backRDrive.setTargetPosition(newBackRTarget);
            robot.slide.setTargetPosition(newArmTarget);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.backLDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.backRDrive.setPower(Math.abs(speed));
            robot.slide.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() || robot.backLDrive.isBusy() || robot.rightDrive.isBusy() || robot.backRDrive.isBusy() || robot.slide.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget, newBackLTarget,  newRightTarget, newBackRTarget, newArmTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.backLDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition(),
                        robot.backRDrive.getCurrentPosition(),
                        robot.slide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.backLDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.backRDrive.setPower(0);
            robot.slide.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
