// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Auto. */
  // fasle = left, true = right
  private boolean headingDir;
  private boolean pastdir;
  private double direction;
  private boolean calledBefore = false;
  private Pose3d currentPose3d;
  private PhotonCamera camera;
  private PhotonCamera camera2;
  private PhotonCamera camera3;
  private ArrayList<PhotonCamera> listOfCameras = new ArrayList<PhotonCamera>();

  private AprilTagFieldLayout aprilTagFieldLayouts = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private Optional<EstimatedRobotPose> mEstimatedRobotPose;
  private ArrayList<Boolean> camerasHaveTargets = new ArrayList<Boolean>();
  private ArrayList<Transform3d> robotToCam = new ArrayList<Transform3d>();

  private static final Transform3d camera1Pos = new Transform3d(new Translation3d(0.07, 0.48, -0.1),
      new Rotation3d(0, 180, 0));
  private static final Transform3d camera2Pos = new Transform3d(new Translation3d(0.45, 0.12, -0.1),
      new Rotation3d(0, 180, 90));
  private static final Transform3d camera3Pos = new Transform3d(new Translation3d(0.14, 0. - 0.21, -0.1),
      new Rotation3d(0, 180, 180));
  private static final Transform3d camera4Pos = new Transform3d(new Translation3d(-0.25, 0.15, -0.1),
      new Rotation3d(0, 180, 270));
  private PhotonPoseEstimator mEstimator;
  private Pose3d lastPose3d;
  private CommandSwerveDrivetrain mDrivetrain;
  private ArrayList<PhotonPoseEstimator> photonPoseEstimators;
  public Vision(CommandSwerveDrivetrain mCommandSwerveDrivetrain) {
    this.mDrivetrain = mCommandSwerveDrivetrain;
    
    camera = new PhotonCamera("frontFacingCamera");
    // camera2 = new PhotonCamera("leftFacingCamera");
    // camera3 = new PhotonCamera("BackFacingCamera");
    listOfCameras.add(0, camera);
    // listOfCameras.add(1, camera2);
    // listOfCameras.add(1,camera3);
    robotToCam.add(0, camera1Pos);
    // robotToCam.add(1, camera2Pos);
    // robotToCam.add(1,camera3Pos);
    camerasHaveTargets.add(0, false);
    // camerasHaveTargets.add(1,false);
    // camerasHaveTargets.add(2,false);
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
    double dR = 0;
    double size = 0;
    // iterating through our list of cameras
    for (int i = 0; i < listOfCameras.size(); i++) {
     
      // setting our photon pose estimator with our robot to cam and april tag field
      // layout
      mEstimator = new PhotonPoseEstimator(aprilTagFieldLayouts, robotToCam.get(i));
      // setting our camera to the camera from the list
      PhotonCamera m = listOfCameras.get(i);
      // checking if it has a target before we try to do anything with it
      if (camerasHaveTargets.get(i) == true) {
        // checks if our fidlical id is present
        // doesnt currently do anything so im wondering if its needed or not for
        // anything
        if (aprilTagFieldLayouts.getTagPose(m.getLatestResult().getBestTarget().getFiducialId()).isPresent()) {
          // updates our current pose with the camera transformation,pose of the id we
          // see,and our camera position on the robot
          currentPose3d = PhotonUtils.estimateFieldToRobotAprilTag(
              m.getLatestResult().getBestTarget().getBestCameraToTarget(),
              aprilTagFieldLayouts.getTagPose(m.getLatestResult().getBestTarget().getFiducialId()).get(),
              robotToCam.get(i));
        }
        // sets ouir
        mEstimatedRobotPose = mEstimator.estimateCoprocMultiTagPose(m.getLatestResult());
        if (mEstimatedRobotPose.isEmpty()) {
          mEstimatedRobotPose = mEstimator.estimateLowestAmbiguityPose(m.getLatestResult());
        }
        // adding our values we calculated to our overall total
        dx += mEstimatedRobotPose.get().estimatedPose.getX();
        dy += mEstimatedRobotPose.get().estimatedPose.getY();
        dz += mEstimatedRobotPose.get().estimatedPose.getZ();
        dR += mEstimatedRobotPose.get().estimatedPose.getRotation().getAngle();
        size += 1;
      }

    }

    // mCommandSwerveDrivetrain.addVisionMeasurement(currentPose3d.toPose2d(),mEstimatedRobotPose.get().timestampSeconds);

    // updating our current pose with our total values then dividing by amount of
    // cameras and checking for anomilies
    if (size != 0) {
      currentPose3d = checkPoseForAnomilies(
          new Pose3d(dx / size, dy / size, dz / size, new Rotation3d(new Rotation2d(dR / size))));
      lastPose3d = currentPose3d;
      // then here we will pass into vision
      mDrivetrain.addVisionMeasurement(currentPose3d.toPose2d(), Timer.getFPGATimestamp());
    }
  }

  /**
   * 
   * @param x Our Array list of cameras which we iterate through and set another
   *          list to true or false based off of whether or not that camera has a
   *          target
   *          if it doesnt have a target sets the other Array list to False
   */
  private void checkallCameras(ArrayList<PhotonCamera> x) {
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

  private boolean yawFixer(Pose3d pp) {
    // this is to fix the yaw and it was just slightly off and annoying hunter
    if (direction == 0) {
      direction = pp.getRotation().getMeasureZ().compareTo(lastPose3d.getRotation().getMeasureZ());
    } else {
      double dd = pp.getRotation().getMeasureZ().compareTo(lastPose3d.getRotation().getMeasureZ());
      if (dd > direction) {
        headingDir = false;
      } else if (dd <= direction) {
        headingDir = true;
      }
      if (pastdir == headingDir) {
        return true;
      } else if (pp.getRotation().getMeasureZ().isNear(lastPose3d.getRotation().getMeasureZ(), 0.12)) {
        return true;
      }

    }
    return false;
  }

  /**
   * 
   * @param p our passed in pose from update pose
   * @return whether or whether not to trust the pose if yes then return the pose
   *         we passed else return past pose
   */
  private Pose3d checkPoseForAnomilies(Pose3d p) {
    if (calledBefore == false) {
      calledBefore = true;
      lastPose3d = p;
      return lastPose3d;
    }
    if (Timer.getFPGATimestamp() < 10) {
      return p;
    }
    /**
     * if not near out last pose by a factor of 10% return the last pose
     * and we are checking against the X,Y and Rotation values
     * Because its called with peridioic which is every 20ms we wont be going fast
     * enough for it to be an issue of moving or changing to much
     */
    if (!p.getMeasureX().isNear(lastPose3d.getMeasureX(), 0.1) ||
        !p.getMeasureY().isNear(lastPose3d.getMeasureY(), 0.1)
        || !yawFixer(p)) {
      return lastPose3d;
    } else {
      return p;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkallCameras(listOfCameras);
    calculatePose();

  }
}
