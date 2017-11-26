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

@Autonomous(name="FinalBlue1")

public class FinalBlue1 extends LinearOpMode {

    /* Declare OpMode members. */
    LZRobot robot = new LZRobot();  // Use a Pushbot's hardware
    LZNavSys nav = new LZNavSys();

    static final double     DRIVE_SPEED             = 0.25;
    static final double     TURN_SPEED              = 0.25;
    double input;
    double prevRangeValue = 0.0;


    //private double input;
  //  private double prevRangeValue = 0.0;

    public void runOpMode() {

        robot.init(hardwareMap, this);
        nav.initVuforia(this, robot);
        input = robot.range1.getDistance(DistanceUnit.CM);

        waitForStart();


        nav.activateTracking();

        robot.s2.setPosition(0);
        //encoderDrive(DRIVE_SPEED, 0, 0, 0, 0, 7, 5);

        sleep(2000);
        if (input > 100.0 && prevRangeValue <= 100.0)
        {
            input = prevRangeValue;
        }
        else
        {
            prevRangeValue = input;
        }

        if(robot.color.blue() > 0){
            robot.moveTime("forwards", DRIVE_SPEED, 200);
            robot.s2.setPosition(0.7);
            robot.moveTime("stop", 0, 1000);
            robot.moveTime("backwards", DRIVE_SPEED, 300);
        } else {
            robot.moveTime("backwards", DRIVE_SPEED, 350);
            robot.s2.setPosition(0.7);
            robot.moveTime("stop", 0, 1000);
            robot.moveTime("forwards", DRIVE_SPEED, 350);
        }
        if (nav.whatRelic() == "LEFT"){
          while (input != 47) {
              robot.move("forwards", DRIVE_SPEED);
          }
            robot.rotateTime("cclock", TURN_SPEED, 300);
            robot.moveTime("forwards", DRIVE_SPEED, 250);
        } else if (nav.whatRelic() == "CENTER"){
            robot.moveTime("forwards", DRIVE_SPEED, 4);
            robot.rotateTime("cclock", TURN_SPEED, 1);
            robot.moveTime("forwards", DRIVE_SPEED, 1);
        } else {
            robot.moveTime("forwards", DRIVE_SPEED, 5);
            robot.rotateTime("cclock", TURN_SPEED, 1);
            robot.moveTime("forwards", DRIVE_SPEED, 1);

        }

        robot.moveRobot(0, 0, 0);




    }
}
