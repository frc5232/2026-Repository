// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Auto. */
  private boolean calledBefore = false;
  private Pose3d startingPose3d;
  private Pose3d goalPose;
  private Pose3d currentPose3d;
  private PhotonCamera camera;
  private PhotonCamera camera2;
  private PhotonCamera camera3;
  private PhotonCamera camera4;
  private List<PhotonTrackedTarget> listOftargetsFromAllCameras;
  private ArrayList<PhotonCamera> listOfCameras = new ArrayList<PhotonCamera>();
  private Transform3d target;
  private AprilTagFieldLayout aprilTagFieldLayouts = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private Optional<EstimatedRobotPose> mEstimatedRobotPose;
  private ArrayList<Boolean> camerasHaveTargets = new ArrayList<Boolean>();
  private ArrayList<Transform3d> robotToCam = new ArrayList<Transform3d>();
  public static final Transform3d camera1Pos=
    new Transform3d(new Translation3d(-0.4, 0.12, -0.1), new Rotation3d(0, 270, 0));
  public static final Transform3d camera2Pos=
    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  public static final Transform3d camera3Pos=
    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  public static final Transform3d camera4Pos=
    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 90, 0));
  private PhotonPoseEstimator mEstimator;
  private Pose3d lastPose3d;
  private CommandSwerveDrivetrain mDrivetrain;
    public Vision(CommandSwerveDrivetrain mCommandSwerveDrivetrain){
    this.mDrivetrain = mCommandSwerveDrivetrain;

    camera = new PhotonCamera("frontFacingCamera");
    listOfCameras.add(0, camera);
    // listOfCameras.add(1, camera2);
    // listOfCameras.add(2, camera3);
    // listOfCameras.add(3, camera4);
    robotToCam.add(0, camera1Pos);
    // robotToCam.add(1, camera2Pos);
    // robotToCam.add(2, camera3Pos);
    // robotToCam.add(3, camera4Pos);
    camerasHaveTargets.add(0,true);
  }
  public void calculatePose(){
    // setting values for us to update with each and every camera values 
    double dx = 0;
    double dy = 0;
    double dz = 0;
    double dR = 0;
    double amb = 0;
    // iterating through our list of cameras
    for(int i = 0; i < listOfCameras.size();i++){
      // setting our photon pose estimator with our robot to cam and april tag field layout
      mEstimator = new PhotonPoseEstimator(aprilTagFieldLayouts, robotToCam.get(i));
      //setting our camera to the camera from the list
      PhotonCamera m = listOfCameras.get(i);
      //checking if it has a targer before we try to do anything with it
      if(camerasHaveTargets.get(i)){
        //checks if our fidlical id is present
        //doesnt currently do anything so im wondering if its needed or not for anything
        if(aprilTagFieldLayouts.getTagPose(camera.getLatestResult().getBestTarget().getFiducialId()).isPresent()){
          // updates our current pose with the camera transformation,pose of the id we see,and our camera position on the robot
          currentPose3d = PhotonUtils.estimateFieldToRobotAprilTag(camera.getLatestResult().getBestTarget().getBestCameraToTarget(), aprilTagFieldLayouts.getTagPose(camera.getLatestResult().getBestTarget().getFiducialId()).get(), robotToCam.get(i));
        }
        //sets ouir 
        mEstimatedRobotPose = mEstimator.estimateCoprocMultiTagPose(camera.getLatestResult());
         if(mEstimatedRobotPose.isEmpty()){
           mEstimatedRobotPose = mEstimator.estimateLowestAmbiguityPose(camera.getLatestResult());
         }
        
        dx += mEstimatedRobotPose.get().estimatedPose.getX();
        dy += mEstimatedRobotPose.get().estimatedPose.getY();
        dz += mEstimatedRobotPose.get().estimatedPose.getZ();
        dR += mEstimatedRobotPose.get().estimatedPose.getRotation().getAngle();
        amb += camera.getLatestResult().getBestTarget().getPoseAmbiguity();
      }

      }
    // SmartDashboard.putNumber("DR", dR/listOfCameras.size());
    // SmartDashboard.putNumber("Dx", dx/listOfCameras.size());
    // SmartDashboard.putNumber("Dy", dy/listOfCameras.size());
    // SmartDashboard.putNumber("Dz", dz/listOfCameras.size());
     SmartDashboard.putNumber("amb", amb);
    /**
     * put into the drivetrain once we have it added
     */

      
    //mCommandSwerveDrivetrain.addVisionMeasurement(currentPose3d.toPose2d(),mEstimatedRobotPose.get().timestampSeconds);
    double size = listOfCameras.size();
    currentPose3d = checkPoseForAnomilies(new Pose3d(dx/size, dy/size, dz/size, new Rotation3d(new Rotation2d(dR/size))),amb/size);
    lastPose3d = currentPose3d;
    // then here we will pass into vision
    mDrivetrain.addVisionMeasurement(currentPose3d.toPose2d(), Timer.getFPGATimestamp());
    //return currentPose3d = mEstimatedRobotPose.get().estimatedPose;
  }
  //checks all cameras to see if they have a target and sets them as such
  private void checkallCameras(ArrayList<PhotonCamera> x){

    // currently there is an index error and a fid false.  just added diagnostices 
    // does x have any elements at all and does  this code execute without error
    //SmartDashboard.putString("checking cameras","");
    //SmartDashboard.putNumber("# of cameras", x.size());

    for(int i =0; i< x.size(); i++){
      if (x.get(i).getLatestResult().hasTargets() == true){
       camerasHaveTargets.set(i, true);
     }else{
        camerasHaveTargets.set(i, false);
     }}
  }
  // this checks our pose we passed in to our last pose and makes sure that it hasn't changed by a wild degree or else it will throw off our auto
  private Pose3d checkPoseForAnomilies(Pose3d p, double amb){
    if(calledBefore == false){
      calledBefore = true;
      lastPose3d = p;
      return p;
    }else{
    if(p.getX() >= lastPose3d.getX() * 1.75 || p.getY() >= lastPose3d.getY() * 1.75 || p.getZ() >= lastPose3d.getZ() * 1.75 || p.getRotation().getAngle() >= lastPose3d.getRotation().getAngle() * 1.75){
      return lastPose3d;
    }else if(amb > 0.25){
      return lastPose3d;
    }else{
      return lastPose3d;
    }}
      
    
  }
  private int checkcameraFiFo(int i){
    for(int j = 0; j <20; j++){
      if(null != listOfCameras.get(i).getAllUnreadResults().get(j)){
        return j;
      }
    }
    return 10;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkallCameras(listOfCameras);
    calculatePose();
    //SmartDashboard.putNumber("list of cameras size", listOfCameras.size());
    //SmartDashboard.putNumber("list of ", camerasHaveTargets.size());
    // SmartDashboard.putNumber("fid", camera.getLatestResult().getBestTarget().getFiducialId());
    //SmartDashboard.putNumber("x value", mDrivetrain.getState().Pose.getX());
    SmartDashboard.putBoolean("calledbefore", calledBefore);
    //dummytest
  }
}
