package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

@TeleOp(name = "NewDriveSys")
public class NEWDRIVESYSTEM extends LinearOpMode {

    LZRobot robot = new LZRobot();
    LZNavSys nav = new LZNavSys();

    final double TARGET_DISTANCE = 400.0;

    public void runOpMode() {

        robot.init(hardwareMap, this);
        nav.initVuforia(this, robot);

        telemetry.addData("Initialization", "Completed");
        telemetry.update();
        robot.s1.setPosition(0.2);

        waitForStart();

        nav.activateTracking();

        while (opModeIsActive()) {

            try {

                if (nav.targetsAreVisible() && gamepad1.left_bumper) {
                    // Calculate automatic target approach
                    nav.cruiseControl(TARGET_DISTANCE);

                } else {
                    // Drive the robot using the joysticks
                    robot.manualDrive();
                }

                // Build telemetry messages with Navigation Information;
                nav.addNavTelemetry();

                robot.moveRobot();

                telemetry.update();

            } catch(IndexOutOfBoundsException e) {

                e.printStackTrace();

            }

        }
    }
}