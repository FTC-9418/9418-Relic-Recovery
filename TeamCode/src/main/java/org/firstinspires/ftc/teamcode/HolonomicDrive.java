package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 *  This is the main Teleop that contains all of the mechanisms
 */

@TeleOp(name="Holonomic Drive", group="Drive")
public class HolonomicDrive extends OpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // Use the class created to define the robot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Set Cube Manipulator to starting values
        robot.naturalServo();
        robot.axis.setPosition(0.036);
        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Test", "Foo Bar Fizz Buzz Xyzzy");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive();
        spin();
        grip();
        slide();
        telemetry.addData("Axis: ", robot.axis.getPosition());
        telemetry.addData("Upper Left: ", robot.ul.getPosition());
        telemetry.addData("Upper Right: ", robot.ur.getPosition());
        telemetry.addData("Lower Left: ", robot.ll.getPosition());
        telemetry.addData("Lower Right: ", robot.lr.getPosition());
    }

    public void drive() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = -gamepad1.right_stick_x;

        int dir = Hardware.Direction_Stop;
        if(y >= 0.5) {
            dir |= Hardware.Direction_Forward;
        } else if(y <= -0.5) {
            dir |= Hardware.Direction_Reverse;
        }
        if(x >= 0.5) {
            dir |= Hardware.Direction_Right;
        } else if(x <= -0.5) {
            dir |= Hardware.Direction_Left;
        }
        if(dir == Hardware.Direction_Stop) {
            if(z >= 0.5) {
                dir |= Hardware.Direction_RotateRight;
            } else if(z <= -0.5) {
                dir |= Hardware.Direction_RotateLeft;
            }
        }
        robot.drive(dir, 1);
    }

    public void spin() {
        double pos = 0.036;
        if(gamepad1.a) {
            robot.axis.setPosition(pos);
        }
        else if(gamepad1.b) {
            robot.axis.setPosition(pos+0.06);
        }
    }

    // Set button activity to false
    boolean xActive = false;
    boolean yActive = false;

    // Set previous and current state of button
    boolean xPrevState = false;
    boolean yPrevState = false;

    public void grip() {

        // Check the status of the buttons
        boolean xCurrState = gamepad2.x;
        boolean yCurrState = gamepad2.y;

        // Check for button state transitions.
        if ((xCurrState == true) && (xCurrState != xPrevState))  {
            // Button is transitioning to a pressed state
            xActive = !xActive;
            if (xActive == true) {
                robot.bottomGrip(true);
            } else {
                robot.bottomGrip(false);
            }
        } else if ((yCurrState == true) && (yCurrState != yPrevState)) {
            // Button is transitioning to a pressed state
            yActive = !yActive;
            if (yActive == true) {
                robot.topGrip(true);
            } else {
                robot.topGrip(false);
            }
        }
        xPrevState = xCurrState;
        yPrevState = yCurrState;
    }

    public void slide() {
        if(gamepad2.right_bumper) {
            robot.slide.setPower(0.8);
        } else if(gamepad2.left_bumper) {
            robot.slide.setPower(-0.5);
        } else {
            robot.slide.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
