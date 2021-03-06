
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ZenithHardAutonomous_V2", group="Zenith")
@Disabled
public class ZenithHardCodingAutonomous2 extends LinearOpMode{ //LinearOpMode
    /* Declare OpMode members. */
    HardwareZenith2          robot   = new HardwareZenith2();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static double     FORWARD_SPEED = 0.2; //initial speeds.
    static double     TURN_SPEED    = 0.2;

    public GoldAlignExample a = new GoldAlignExample();
    public GoldAlignDetector aDetector = a.getDetector();


    @Override
    public void runOpMode() {

        /////////////////////////
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        aDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        aDetector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        aDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        aDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        aDetector.downscale = 0.4; // How much to downscale the input frames

        aDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        aDetector.maxAreaScorer.weight = 0.005; //

        aDetector.ratioScorer.weight = 5; //
        aDetector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        aDetector.enable(); // Start the detector!
        /////////////////////////


        telemetry.addData("IsAligned" , aDetector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , aDetector.getXPosition()); // Gold X position.

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

///////////////// Autonomous Mode Code /////////// Don't Touch Stuff Above /////////////////////////////////////
        //Drop down
        robot.dropServo.setPosition(0);
        sleep(2500);

        //Moves forwards
        this.robotForwards(0.5, 0.4);
        this.robotHardStop();

        //Rotates to the left
        FORWARD_SPEED = 0.5;
        robot.backLeftDrive.setPower(-FORWARD_SPEED);
        robot.backRightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Pause briefly --- this might solve the error ---  NEW CODE!!!
        FORWARD_SPEED = 0.0;
        robot.backLeftDrive.setPower(-FORWARD_SPEED);
        robot.backRightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Rotate Right until cube is aligned
        while(!aDetector.getAligned()){ //while(aDetector.getXPosition() != 175){
            FORWARD_SPEED = 0.25;
            robot.backLeftDrive.setPower(FORWARD_SPEED);
            robot.backRightDrive.setPower(-FORWARD_SPEED);
            //runtime.reset();
        }
        //Rotate left a little more to offset the position being slightly off
        this.robotRotateLeft(0.05,0.1);

        //Move towards the cube
        if (aDetector.getAligned()){ //if (aDetector.getXPosition() == 175){
            robotForwards(6, 0.3);
            robotHardStop();
        }

        //Reset


        //Next task here...


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

        //Stop the detector:
            //aDetector.disable();
    }


    public void robotForwards(double t, double speed){ //t is in sec.
        FORWARD_SPEED = speed;
        robot.backLeftDrive.setPower(FORWARD_SPEED);
        robot.backRightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Driving to Crater: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.backLeftDrive.setPower(0.0);
        robot.backRightDrive.setPower(0.0);
    }

    public void robotHardStop(){
        robot.backLeftDrive.setPower(-0.1);
        robot.backRightDrive.setPower(-0.1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Driving to Crater: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.backLeftDrive.setPower(0.0);
        robot.backRightDrive.setPower(0.0);
    }

    public void robotReverse(double t, double speed){ //t is in sec.
        FORWARD_SPEED = speed;
        robot.backLeftDrive.setPower(-FORWARD_SPEED);
        robot.backRightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void robotRotateRight(double t, double speed){ //t is in sec.
        FORWARD_SPEED = speed;
        robot.backLeftDrive.setPower(FORWARD_SPEED);
        robot.backRightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void robotRotateLeft(double t, double speed){ //t is in sec.
        FORWARD_SPEED = speed;
        robot.backLeftDrive.setPower(-FORWARD_SPEED);
        robot.backRightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

}
