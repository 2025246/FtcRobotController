package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive")
public class Drive extends OpMode {
    private DcMotor leftFront;  // 0
    private DcMotor leftRear;   // 2
    private DcMotor rightFront; // 1
    private DcMotor rightRear;  // 3
    private DcMotor intake;
    private DcMotor lift;

    /*private Servo intake;
    private DcMotor lift;
    public static final double MID_SERVO       =  0.5 ;
    public static final double LIFT_UP_POWER    =  0.45 ;
    public static final double LIFT_DOWN_POWER  = -0.45 ;
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo */

    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        intake = hardwareMap.get(DcMotor.class,"intake");
        lift = hardwareMap.get(DcMotor.class,"lift");

        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);

        int pos = 0;
        int pos2 = 0;

        telemetry.addData("Say", "Hello Driver");
    }
    @Override
    public void loop() {
        double left;
        double right;
        double rf = -gamepad1.right_stick_y -gamepad1.right_stick_x -gamepad1.left_stick_x;
        double rr = -gamepad1.right_stick_y + gamepad1.right_stick_x -gamepad1.left_stick_x;
        double lf = gamepad1.right_stick_y + gamepad1.right_stick_x  -gamepad1.left_stick_x;
        double lr = gamepad1.right_stick_y -gamepad1.right_stick_x -gamepad1.left_stick_x;
        if(Math.abs(rf) >= 0.1 || Math.abs(rr) >= 0.1 || Math.abs(lf) >= 0.1 || Math.abs(lr) >= 0.1){
            rightFront.setPower(rf);
            rightRear.setPower(rr);
            leftRear.setPower(lr);
            leftFront.setPower(lf);
        }
        else{
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftRear.setPower(0);
            leftFront.setPower(0);
        }

        intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        lift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        /*-------_-------\\
        if (gamepad2.a) {
            _.setPower(1);
        } else if (gamepad2.b) {
            _.setPower(-1);
        } else  {
            _.setPower(0);
        }

        //-------_-------\\
        if (gamepad1.a) {
            _.setPower(1);
        } else if (gamepad1.b) {
            _.setPower(-1);
        } else  {
            _.setPower(0);
        }

        //-------_-------\\
        if (gamepad2.a) {
            _.setPosition(1);
        } else if (gamepad2.b) {
            _.setPosition(-1);
        } else  {
            _.setPosition(0);
        } */
    }

    /* Code to run ONCE after the driver hits STOP */

    @Override
    public void stop() {
    }
}
