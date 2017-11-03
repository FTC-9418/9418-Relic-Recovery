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
        //robot.naturalServo();
        robot.axis.setPower(0);

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
        robot.axis.setPower(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive();
        //grip();
        //telemetry.addData("Axis: ", robot.axis.getPosition());
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
    /*public void grip() {
        if(gamepad1.a) {
            robot.bottomGrip();
        }
        else if(gamepad1.y) {
            // Release top
        }

    }*/

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
