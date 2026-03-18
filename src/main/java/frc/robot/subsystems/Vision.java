// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private boolean calledBefore = false;
  private Pose3d currentPose3d;
  private PhotonCamera camera;

  private ArrayList<PhotonCamera> listOfCameras = new ArrayList<PhotonCamera>();

  private AprilTagFieldLayout aprilTagFieldLayouts = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private Optional<EstimatedRobotPose> estimatedRobotPose;
  private ArrayList<Boolean> camerasHaveTargets = new ArrayList<Boolean>();
  private ArrayList<Transform3d> robotToCam = new ArrayList<Transform3d>();

  private static final Transform3d camera1Pos = Constants.visionConstant.camera1Pos;

  private PhotonPoseEstimator estimator;
  private Pose3d lastPose3d;
  private CommandSwerveDrivetrain drivetrain;
  private ArrayList<PhotonPoseEstimator> photonPoseEstimators =  new ArrayList<>();

  public Vision(CommandSwerveDrivetrain mCommandSwerveDrivetrain) {
    this.drivetrain = mCommandSwerveDrivetrain;

    camera = new PhotonCamera("frontFacingCamera");
      
    listOfCameras.add(0, camera);

    robotToCam.add(0, camera1Pos);

    camerasHaveTargets.add(0, false);

    photonPoseEstimators.add(new PhotonPoseEstimator(aprilTagFieldLayouts, camera1Pos));

  }

  /**
   * Calculating and updateing our pose and passing it to the drivetrain which is
   * needed to for the auto
   */
  public void calculatePose() {
    // setting values for us to update with each and every camera values
    double dx = 0;
    double dy = 0;
    double dz = 0;
    
    double size = 0;
    // iterating through our list of cameras
    for (int i = 0; i < listOfCameras.size(); i++) {

      // setting our photon pose estimator with our robot to cam and april tag field
      // layout
      estimator = photonPoseEstimators.get(i);
      // setting our camera to the camera from the list
      PhotonCamera m = listOfCameras.get(i);
      // checking if it has a target before we try to do anything with it
      if (camerasHaveTargets.get(i) == true) {
        // checks if our fidlical id is present
        // doesnt currently do anything so im wondering if its needed or not for
        // anything

        estimatedRobotPose = estimator.estimateCoprocMultiTagPose(m.getLatestResult());
        if (estimatedRobotPose.isEmpty()) {
          estimatedRobotPose = estimator.estimateLowestAmbiguityPose(m.getLatestResult());
        }
        // adding our values we calculated to our overall total
        dx += estimatedRobotPose.get().estimatedPose.getX();
        dy += estimatedRobotPose.get().estimatedPose.getY();
        dz += estimatedRobotPose.get().estimatedPose.getZ();
        
        size += 1;
      }

    }

    // mCommandSwerveDrivetrain.addVisionMeasurement(currentPose3d.toPose2d(),mEstimatedRobotPose.get().timestampSeconds);

    // updating our current pose with our total values then dividing by amount of
    // cameras and checking for anomilies
    if (size != 0) {
      if (checkPoseForAnomilies(
          new Pose3d(dx / size, dy / size, dz / size,drivetrain.getRotation3d()))) {

        lastPose3d = currentPose3d;
        drivetrain.addVisionMeasurement(currentPose3d.toPose2d(), camera.getLatestResult().getTimestampSeconds());
      } else {
        drivetrain.addVisionMeasurement(lastPose3d.toPose2d(), camera.getLatestResult().getTimestampSeconds());
      }

    }
  }

  /**
   * 
   * @param x Our Array list of cameras which we iterate through and set another
   *          list to true or false based off of whether or not that camera has a
   *          target
   *          if it doesnt have a target sets the other Array list to False
   */
  private void checkAllCameras(ArrayList<PhotonCamera> x) {
    // iterating through the cameras and checking if they have targets if they do
    // updating it to true else make it false
    for (int i = 0; i < x.size(); i++) {
      if (x.get(i).getLatestResult().hasTargets() == true) {
        camerasHaveTargets.set(i, true);
      } else {
        camerasHaveTargets.set(i, false);
      }
    }
  }

  

  /**
   * 
   * @param p our passed in pose from update pose
   * @return whether or whether not to trust the pose if yes then return the pose
   *         we passed else return past pose
   */
  private boolean checkPoseForAnomilies(Pose3d p) {
    if (calledBefore == false) {
      calledBefore = true;
      lastPose3d = p;
      return true;
    }
    if (Timer.getFPGATimestamp() < 5) {
      return true;
    }
    /**
     * if not near out last pose by a factor of 10% return the last pose
     * and we are checking against the X,Y and Rotation values
     * Because its called with peridioic which is every 20ms we wont be going fast
     * enough for it to be an issue of moving or changing to much
     */
    if (!p.getMeasureX().isNear(lastPose3d.getMeasureX(), 0.1) ||
        !p.getMeasureY().isNear(lastPose3d.getMeasureY(), 0.1)
        ) {
      return false;
    } else {
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkAllCameras(listOfCameras);
    calculatePose();

  }
}
