
package pedroPathing.examples;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Mecanam Wheel Test", group="Iterative OpMode")

public class BasicMecham extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*

        frontLeft  = hardwareMap.get(DcMotor.class, "leftFront");//0
        backLeft  = hardwareMap.get(DcMotor.class, "leftBack");//1

        frontRight = hardwareMap.get(DcMotor.class, "rightFront");//2
        backRight = hardwareMap.get(DcMotor.class, "rightBack");//3

        //since the motors are 180 out one side needs to be reversed
        //!!!frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // the front right gearbox is mounted wrong. Only way to get going same direction
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

*/
        frontLeft  = hardwareMap.get(DcMotor.class, "leftFront");//0
        backRight  = hardwareMap.get(DcMotor.class, "leftBack");//1

        frontRight = hardwareMap.get(DcMotor.class, "rightFront");//2
        backLeft = hardwareMap.get(DcMotor.class, "rightBack");//3

        //since the motors are 180 out one side needs to be reversed
        //!!!frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // the front right gearbox is mounted wrong. Only way to get going same direction
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower;
        double backLeftPower;

        double frontRightPower;
        double backRightPower;

        double y = gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn),1);

        frontLeftPower = (y + x + turn) / denominator;
        backLeftPower  = (y - x + turn) / denominator;

        frontRightPower = (y - x - turn) / denominator;
        backRightPower  = (y + x - turn) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);

        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left", "front (%.2f), back (%.2f)", frontLeftPower, backLeftPower);
        telemetry.addData("Right", "front (%.2f), back (%.2f)", frontRightPower, backRightPower );
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}