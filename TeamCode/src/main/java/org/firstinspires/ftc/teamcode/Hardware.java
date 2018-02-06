package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * This class defines all of the robot hardware
 *
 */
public class Hardware {

    // Holonomic Drivetrain Motors
    public DcMotor fl = null; // Front Left motor
    public DcMotor fr = null; // Front Right motor
    public DcMotor bl = null; // Back Left motor
    public DcMotor br = null; // Back Right motor

    // Cube Manipulator
    public Servo   gl    = null; // Grip Left servo
    public Servo   gr    = null; // Grip Right servo
    public DcMotor slide = null; // Linear Slide motor

    // Jewel Manipulator
    public Servo       sTs = null; // Side to Side servo (Moves sTs relative to robot front)
    public Servo       fTb = null; // Front to Back servo (Moves fTb relative to robot front)
    public ColorSensor cs  = null; // Color Sensor

    // Linear Slide
    public DcMotor sw = null; // Slide Winch
    public Servo   sg = null; // Slide Grabber
    public Servo   sr = null; // Slide Rotation

    /* local OpMode members */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  =  new ElapsedTime();

    /* Constructor */
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Drivetrain Motors
        fr = hwMap.dcMotor.get("FR");
        fl = hwMap.dcMotor.get("FL");
        br = hwMap.dcMotor.get("BR");
        bl = hwMap.dcMotor.get("BL");

        // Define and Initialize Cube Manipulator
        gl    = hwMap.servo.get("GL");
        gr    = hwMap.servo.get("GR");
        slide = hwMap.dcMotor.get("SLIDE");

        // Define and Initialize Linear Slide
        sw = hwMap.dcMotor.get("SW");
        sg = hwMap.servo.get("SG");
        sr = hwMap.servo.get("SR");

        // Jewel Manipulator
        sTs = hwMap.servo.get("STS");
        fTb = hwMap.servo.get("FTB");
        cs  = hwMap.colorSensor.get("CS");

        // Set all motors to zero power
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        slide.setPower(0);
        sw.setPower(0);

        // Set motors to run without encoders.
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor direction
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        // Set color sensor light to on
        cs.enableLed(true);
    }

    //  Rotate, Left, Right, Back, Forward
    public final static int Direction_Stop         = 0;
    public final static int Direction_Forward      = 0b00001;
    public final static int Direction_Reverse      = 0b00010;
    public final static int Direction_Left         = 0b01000;
    public final static int Direction_Right        = 0b00100;
    public final static int Direction_Rotate       = 0b10000;
    public final static int Direction_RotateLeft   = Direction_Rotate  | Direction_Left;
    public final static int Direction_RotateRight  = Direction_Rotate  | Direction_Right;
    public final static int Direction_ForwardLeft  = Direction_Forward | Direction_Left;
    public final static int Direction_ForwardRight = Direction_Forward | Direction_Right;
    public final static int Direction_ReverseLeft  = Direction_Reverse | Direction_Left;
    public final static int Direction_ReverseRight = Direction_Reverse | Direction_Right;

    public void drive(int dir, double pwr){

        switch (dir) {
            case Direction_Stop         : motorPower(0,0,0,0); break;
            case Direction_Forward      : motorPower(pwr,pwr,pwr,pwr);                   break;
            case Direction_Reverse      : motorPower(-pwr,-pwr,-pwr,-pwr);               break;
            case Direction_Right        : motorPower(-pwr,pwr,pwr,-pwr);                 break;
            case Direction_Left         : motorPower(pwr,-pwr,-pwr,pwr);                 break;
            case Direction_RotateRight  : motorPower(pwr,-pwr,pwr,-pwr);                 break;
            case Direction_RotateLeft   : motorPower(-pwr,pwr,-pwr,pwr);                 break;
            case Direction_ForwardLeft  : motorPower(pwr,0,0,pwr);          break;
            case Direction_ForwardRight : motorPower(0,pwr,pwr,0);          break;
            case Direction_ReverseRight : motorPower(-pwr,0,0,-pwr);        break;
            case Direction_ReverseLeft  : motorPower(0,-pwr,-pwr,0);        break;
            default                     : motorPower(0,0,0,0); break;

        }
    }

    public void stop() {
        motorPower(0,0,0,0);
    }

    public void motorPower(double frpwr, double flpwr, double brpwr, double blpwr) {
        fr.setPower(frpwr);
        fl.setPower(flpwr);
        br.setPower(brpwr);
        bl.setPower(blpwr);
    }

    // Sets servos to go to mirrored position
    public void normalizeServo(Servo servo, double position) {
        if(servo==gl){
            servo.setPosition(position);
        }
        else {
            servo.setPosition(1-position);
        }
    }

    // Grip cube manipulator
    public void grip(boolean active, double pos) {
        if(active==true) {
            normalizeServo(gl,0.01);
            normalizeServo(gr,0.01);
        } else if(active==false) {
            normalizeServo(gl,pos);
            normalizeServo(gr,pos);
        }
    }

    public final double NeutralSlideGrabPos = 1.0;
    public final double SlideGrabPos = 0.5;
    public final double NeutralSlideRotatePos = 1.0;
    public final double SlideRotatePos = 0.3;

    // Sets servos to start position
    public void startServo() {
        // Set 0 position of cube manipulator servos
        normalizeServo(gl, 0.7);
        normalizeServo(gr, 0.65);

        // Set 0 position of jewel manipulator servos
        fTb.setPosition(0.39); // 0 position
        sTs.setPosition(0.63); // 0 position

        sg.setPosition(NeutralSlideGrabPos);
        sr.setPosition(NeutralSlideRotatePos);
    }

    // Sets servos to "natural" position
    public void naturalServo() {
        // Set 0 position of cube manipulator servos
        normalizeServo(gl, 0.53);
        normalizeServo(gr, 0.43);

        sg.setPosition(NeutralSlideGrabPos);
        sr.setPosition(NeutralSlideRotatePos);
    }

    public void stepServo(Servo servo, double endpoint) {
        double currPos = servo.getPosition();
        if(Double.isNaN(currPos)) {
            servo.setPosition(endpoint);
            return;
        }
        double delta = (endpoint - currPos)/10;
        for(int i=0; i<10; i++) {
            servo.setPosition(clip(currPos + (delta*i)));
            sleep(100);
        }
    }

    public double clip(double pos) {
        return Math.max(Math.min(pos, 1.0), 0);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

