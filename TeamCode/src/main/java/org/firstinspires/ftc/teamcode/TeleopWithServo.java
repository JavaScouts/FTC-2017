package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kyle on 9/13/2015.
 */
public class TeleopWithServo extends OpMode {

    final double servoPosition = 1.0;
    final double servoPosition2 = 0;
    final double rawr = 1.0;
    final double rawr2 = 0;
    final double quack = 1.0;
    final double quack2 = 0.25;

    DcMotor leftmotor;
    DcMotor rightmotor;
    DcMotor motor1;
    DcMotor motor2;
    Servo climber1;
    Servo climber2;
    Servo servo4;



    public void init() {

        leftmotor = hardwareMap.dcMotor.get("left_drive");
        rightmotor = hardwareMap.dcMotor.get("right_drive");
        motor1 = hardwareMap.dcMotor.get("leftdrive");
        motor2 = hardwareMap.dcMotor.get("rightdrive");
        climber1 = hardwareMap.servo.get("climber1");
        climber2 = hardwareMap.servo.get("climber2");
        servo4 = hardwareMap.servo.get("servo4");
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        leftmotor.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.FORWARD);

    }

    public void loop() {

        if (gamepad2.y) {
            climber2.setPosition(servoPosition2);
        }

        else  {
            climber2.setPosition(servoPosition);
        }


        if (gamepad2.x) {
            climber1.setPosition(rawr);
        }

        else {
            climber1.setPosition(rawr2);
      }

        if (gamepad2.b) {
          servo4.setPosition(quack2);
        }

        else {
            servo4.setPosition(quack);
        }

        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        leftmotor.setPower(leftY * 0.5);
        motor1.setPower(leftY * 0.5);
        rightmotor.setPower(rightY * 0.6);
        motor2.setPower(rightY * 0.6);


    }
}