package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by Liam on 11/20/2017.
 */

public class StrafeTest extends OpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    private final double ticksPerRevolution = 1120;
    private double prevTime;
    private int prevflEncoderPosition;
    private int prevfrEncoderPosition;
    private int prevblEncoderPosition;
    private int prevbrEncoderPosition;

    private final double drivePidKp = 1;    // Tuning variable for PID.
    private final double drivePidTi = 1.0;  // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;  // Account for error in 0.1 sec.
    private final double drivePidIntMax = 0.75;  // Limit to max speed.

    private Pid flDrive = null;
    private Pid frDrive = null;
    private Pid blDrive = null;
    private Pid brDrive = null;

    public void init() {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        prevTime = 0;
        prevflEncoderPosition = fl.getCurrentPosition();
        prevfrEncoderPosition = fr.getCurrentPosition();
        prevblEncoderPosition = bl.getCurrentPosition();
        prevbrEncoderPosition = br.getCurrentPosition();

        flDrive = new Pid(drivePidKp, drivePidTi, drivePidTd, -drivePidIntMax, drivePidIntMax);
        frDrive = new Pid(drivePidKp, drivePidTi, drivePidTd, -drivePidIntMax, drivePidIntMax);
        blDrive = new Pid(drivePidKp, drivePidTi, drivePidTd, -drivePidIntMax, drivePidIntMax);
        brDrive = new Pid(drivePidKp, drivePidTi, drivePidTd, -drivePidIntMax, drivePidIntMax);

    }

    public void loop() {

        double deltaTime = time - prevTime;
        double flSpeed = (fl.getCurrentPosition() - prevflEncoderPosition) /
                deltaTime;
        double frSpeed = (fr.getCurrentPosition() - prevfrEncoderPosition) /
                deltaTime;
        double blSpeed = (bl.getCurrentPosition() - prevblEncoderPosition) /
                deltaTime;
        double brSpeed = (br.getCurrentPosition() - prevbrEncoderPosition) /
                deltaTime;

        prevTime = time;
        prevflEncoderPosition = fl.getCurrentPosition();
        prevfrEncoderPosition = fr.getCurrentPosition();
        prevblEncoderPosition = bl.getCurrentPosition();
        prevbrEncoderPosition = br.getCurrentPosition();


    }


}
