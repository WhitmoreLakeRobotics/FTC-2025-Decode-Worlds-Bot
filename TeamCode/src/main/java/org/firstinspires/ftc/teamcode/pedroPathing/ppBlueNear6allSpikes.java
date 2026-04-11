package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Autonomous(name = "ppAutonbase", group = "PP")
public class ppBlueNear6allSpikes extends OpMode {

    Robot robot = new Robot();


    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    private stage currentStage = stage._00_unknown;
    private ElapsedTime runtime = new ElapsedTime();

    public static Follower follower;
    public static Pose startPose = new Pose(10, 10, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(15, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP = new Pose(20, 20, Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(25, 25, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(20, 20, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(1, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));

    public static Pose gatePose = new Pose(15, 65, Math.toRadians(180));

    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a, grabPickup2b, scorePickup2, goEndPose, goEndPose2, endPath;
    private PathChain cyclePickup1,spikeB2, interruptedPickup, CornerPickup, Park;


    public void buildPaths() {
        cyclePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())

                .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())

                .addPath(new BezierCurve(pickup1bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePose.getHeading())

                .build();
    }

    @Override
    public void init() {

// NAJ CompBotConstants is a file/class that contains the definition of the gryo and drive motors among other things
        //super.init();
        follower = CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(startPose);
        follower.update();
//  pedroPanelsTelemetry.init();
        Drawing.init();
        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

// disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
// Feedback to Driver Hub for debugging
        updateTelemetry();

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
    }


    @Override
    public void init_loop() {
        //super.init_loop();


        robot.init_loop();

    }

    @Override
    public void start () {
        //super.start();
        robot.start();
    }

    @Override
    public void loop() {

        updateTelemetry();
        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _00_unknown:
                currentStage = stage._10_preStart;
                break;

            case _10_preStart:
                currentStage = stage._20_PreLaunch;
                break;

            case _20_PreLaunch:
                AreYouSure(stage._30_Launch1);

                break;

            case _30_Launch1:
                if (!follower.isBusy()) {
                    dolaunch_process();

                    currentStage = stage._40_PickupSpike1;
                }

                break;

            case _40_PickupSpike1:
                if (runtime.milliseconds() > 500 || robot.sensors.NoArtifacts) { //add sensors here
                    endlaunch_process();
                    follower.followPath(cyclePickup1);

                    currentStage = stage._50_Prelaunch2;
                }

                break;

            case _50_Prelaunch2:
                AreYouSure(stage._60_Launch2);

                break;

            case _60_Launch2:
                if(!follower.isBusy()){
                    dolaunch_process();

                    currentStage = stage._20_PreLaunch;
                }


                break;

            case _70_PickupSpike2Gate:
                if (runtime.milliseconds() > 500 || robot.sensors.NoArtifacts) { //add sensors here
                    endlaunch_process();
                   // follower.followPath(); fix this

                    currentStage = stage._80_PreLaunch3;
                }

                break;

            case _80_PreLaunch3:
                AreYouSure(stage._90_Launch3);

                break;

            case _90_Launch3:
                if(!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._100_Gate2;
                }

            case _100_Gate2:
                if (runtime.milliseconds() > 500 || robot.sensors.NoArtifacts) { //add sensors here
                    endlaunch_process();
                   // follower.followPath(); fix this

                    currentStage = stage._110_PickupSpike3;
                }

                break;

            case _110_PickupSpike3:
             //   follower.followPath(); fix this
                currentStage = stage._120_Gate3;

                break;

            case _120_Gate3:
                currentStage = stage._130_PreLaunch4;

                break;

            case _130_PreLaunch4:
                AreYouSure(stage._140_Launch4);

                break;

            case _140_Launch4:
                if(!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._150_ParkToBeContinued;
                }

                break;

            case _150_ParkToBeContinued:
                if (runtime.milliseconds() > 500 || robot.sensors.NoArtifacts) { //add sensors here
                    endlaunch_process();
                   // follower.followPath(); fix this

                    currentStage = stage._250_end;
                }

                break;

            case _250_end:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    stop();
                    runtime.reset();
                }
                break;



        }

    }



    @Override
    public void stop () {
        //super.stop();
        robot.stop();
    }
    private enum stage {

        _00_unknown,
        _10_preStart,
        _20_PreLaunch,
        _30_Launch1,
        _40_PickupSpike1,
        _50_Prelaunch2,
        _60_Launch2,
        _70_PickupSpike2Gate,
        _80_PreLaunch3,
        _90_Launch3,
        _100_Gate2,
        _110_PickupSpike3,
        _120_Gate3,
        _130_PreLaunch4,
        _140_Launch4,
        _150_ParkToBeContinued,
        _250_end;


    }


    private void updateTelemetry () {
        telemetryMU.addData("Follower Busy?", follower.isBusy());
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //  telemetryMU.addData("LAST Pose", lastPose);
        //  telemetryMU.addData("Current Target Pose", currentTargetPose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P);
        telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D);
        telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F);
        telemetryMU.addData("CONSTRAINTS", "");
        telemetryMU.addData("Tvalue (% complete)", follower.pathConstraints.getTValueConstraint());
        telemetryMU.addData("Current tValue", follower.getCurrentTValue());
        telemetryMU.addData("Velocity Constraint", follower.pathConstraints.getVelocityConstraint());
        telemetryMU.addData("Current Velocity", follower.getVelocity());
        telemetryMU.addData("Trans constraint", follower.pathConstraints.getTranslationalConstraint());
        // telemetryMU.addData("current Trans", follower.getTranslationalError());
        telemetryMU.addData("Heading Constraint", follower.pathConstraints.getHeadingConstraint());

        telemetryMU.update();
        Drawing.drawDebug(follower);
    }

    private void dolaunch_process(){

        robot.launcherBlocker.cmdUnBlock();
        robot.transitionRoller.cmdSpin();
        robot.intake.cmdFoward();
        runtime.reset();

    }

    private void endlaunch_process(){

        robot.launcherBlocker.cmdBlock();
        robot.autoRPM.Measure = false;
        robot.launcher.cmdStop();

    }


    private void AreYouSure(ppBlueNear6allSpikes.stage NextStage){

        //telemetryMU.addData("pathPose2", pickup1bPose);
        //telemetryMU.addData("scorePose", scorePoseAP);
        if (follower.isBusy()) { //we are still running path
//telemetryMU.addData("check intake status", robot.intake.AtIntakeStop); intake.AtIntakeStop is never set to false
            if (robot.sensors.allFilled) {
                telemetryMU.addLine("Intake stopped - break follower");
                // we've got 3 artifacts, stop the path and return to scorePose
                follower.breakFollowing();
                newPath();
                //   robot.autoRPM.Measure = true; // start fly wheels
                robot.autoRPM.Measure = true;
                currentStage = NextStage;
                runtime.reset();

            } else if (follower.getCurrentTValue() > 0.75) { //the path is almost done
                //  robot.autoRPM.Measure = true; //start fly wheels
                robot.autoRPM.Measure = true;
            }
        } else {// path is complete we are back at scorePose move to launch
            robot.autoRPM.Measure = true;
            currentStage = NextStage;
        }

    }

    private  void newPath(){
        interruptedPickup = follower.pathBuilder()
                .addPath (new BezierLine(follower.getPose(), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        follower.followPath(interruptedPickup,true);

    }



}
