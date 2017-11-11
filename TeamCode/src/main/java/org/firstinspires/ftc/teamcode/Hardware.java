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
    public Servo   axis  = null; // Axis servo
    public Servo   test  = null; // Test Servo
    public Servo   ul    = null; // Upper Left servo
    public Servo   ur    = null; // Upper Right servo
    public Servo   ll    = null; // Lower Left servo
    public Servo   lr    = null; // Lower Right servo
    public DcMotor slide = null; // Linear Slide motor

    // Jewel Manipulator
    public Servo       sTs = null; // Side to Side servo
    public Servo       fTb = null; // Front to Back servo
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
        axis  = hwMap.servo.get("AXIS");
        ul    = hwMap.servo.get("UL");
        ur    = hwMap.servo.get("UR");
        ll    = hwMap.servo.get("LL");
        lr    = hwMap.servo.get("LR");
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
        if(servo==ul || servo==ll){
            servo.setPosition(position);
        }
        else {
            servo.setPosition(1-position);
        }
    }

    // Grip from RELATIVE bottom of cube manipulator
    public void bottomGrip(boolean active) {
        boolean inverted = axis.getPosition() > 0.06 ? true : false;
        if(active==true) {
            if(inverted) {
                normalizeServo(ul,0.02);
                normalizeServo(ur,0.02);
            } else {
                normalizeServo(ll,0.02);
                normalizeServo(lr,0.02);
            }
        } else if(active==false) {
            if(inverted) {
                normalizeServo(ul,0.2);
                normalizeServo(ur,0.2);
            } else {
                normalizeServo(ll,0.2);
                normalizeServo(lr,0.2);
            }
        }
    }

    // Grip from RELATIVE top of cube manipulator
    public void topGrip(boolean active) {
        boolean inverted = axis.getPosition() > 0.06 ? true : false;
        if(active==true) {
            if(inverted) {
                normalizeServo(ll,0.03);
                normalizeServo(lr,0.03);
            } else {
                normalizeServo(ul,0.03);
                normalizeServo(ur,0.03);
            }
        } else if(active==false) {
            if(inverted) {
                normalizeServo(lr,0.2);
                normalizeServo(ll,0.2);
            } else {
                normalizeServo(ul,0.2);
                normalizeServo(ur,0.2);
            }
        }
    }

    // Sets servos to "natural" position
    public void naturalServo() {
        // Set 0 position of cube manipulator servos
        normalizeServo(ul, 0.53);
        normalizeServo(ur, 0.53);
        normalizeServo(ll, 0.53);
        normalizeServo(lr, 0.53);
        axis.setPosition(0.036);

        // Set 0 position of jewel manipulator servos
        fTb.setPosition(0.39); // 0 position
        sTs.setPosition(0.4); // 0 position
    }

    public void hitRedJewel() {
        fTb.setPosition(0.7);
    }

    public void hitBlueJewel() {
        fTb.setPosition(-0.7);
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

