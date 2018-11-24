package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="GoldAlign Example", group="DogeCV")
public class GoldAlignExample extends OpMode
{
    // Detector object
    public GoldAlignDetector myDetector;

            public GoldAlignExample(){
                myDetector = new GoldAlignDetector(); // Create detector
            }

            public GoldAlignDetector getDetector(){
                return myDetector;
            }

    @Override
    public void init() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
//        myDetector = new GoldAlignDetector(); // Create detector
        myDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        myDetector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        myDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        myDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        myDetector.downscale = 0.4; // How much to downscale the input frames

        myDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        myDetector.maxAreaScorer.weight = 0.005; //

        myDetector.ratioScorer.weight = 5; //
        myDetector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        myDetector.enable(); // Start the detector!

    }

     //Code to run REPEATEDLY when the driver hits INIT
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
    }

     //Code to run REPEATEDLY when the driver hits PLAY
     //THIS IS THE IMPORTANT STUFF

    @Override
    public void loop() {
        telemetry.addData("IsAligned" , myDetector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , myDetector.getXPosition()); // Gold X position.
    }

     //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Disable the detector
        myDetector.disable();
    }

}