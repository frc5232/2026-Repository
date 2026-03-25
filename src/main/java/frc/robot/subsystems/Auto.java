
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Auto extends SubsystemBase {
  /** Creates a new Auto. */
  private double time = 0;
  private double xSpeed = 0;
  private double ySpeed = 0;
  private double rSpeed = 0;
  private ArrayList<Translation2d> mInteriorPoints;
  private boolean firstRunOfTraj = false;
  private Pose2d goalPose2d;
  private Pose2d currentPose2d;
  private SwerveRequest.FieldCentric drive;
  private CommandSwerveDrivetrain drivetrainAuto;
  private TrajectoryConfig trajectoryConfig;
  private LTVUnicycleController ltvController;
  private ChassisSpeeds chassisSpeeds;
  public List<Translation2d> mList;
  public ArrayList<Translation2d> mArrayList = new ArrayList<>();
  private boolean set;
  private int i;

  public Auto(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric mCentric) {
    drivetrainAuto = drivetrain;
    drive = mCentric;
    currentPose2d = drivetrainAuto.getState().Pose;
    set = false;
    // we can tune this some more when we are at arc field
    // this is for path of the auto from start to where we want to be
    ltvController = new LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0), 0.02, 9);
    goalPose2d = new Pose2d();
  }

  /**
   * Our method to update our pose and needs to be called in peridioic
   */
  private void updatePose() {
    currentPose2d = drivetrainAuto.getState().Pose;
  }

  /**
   * @param c - is our current pose - most likely pass current vision measurement
   *          into it
   * @param g - is our goal pose
   */
  private Trajectory trejGen(Pose2d c, Pose2d g) {
    trajectoryConfig = new TrajectoryConfig(1, 1);

    /**
     * might end up having the list of get passed in but for now we are just having
     * the point to go through be the starting point
     */
    SmartDashboard.putNumber("interior waypoint x", mInteriorPoints.get(0).getX());
    Trajectory extrajectory = TrajectoryGenerator.generateTrajectory(c, mInteriorPoints, g, trajectoryConfig);
    time = Timer.getFPGATimestamp();
    return extrajectory;
  }

  /**
   * 
   * @param tj   our trajectory generated at the start of auto by pick auto to run
   * @param cont our controller as defined in method header
   * @return a chassis speeds which is called in peridoic to update the speeds and
   *         rotations of our drivetrain
   */
  private ChassisSpeeds mGenSpeeds(Trajectory tj, LTVUnicycleController cont) {

    chassisSpeeds = cont.calculate(currentPose2d, tj.sample(Timer.getFPGATimestamp() - time));

    return chassisSpeeds;

  }
  private Commands followAuto(ChassisSpeeds mSpeeds){
    rSpeed = mSpeeds.omegaRadiansPerSecond;
    xSpeed = mSpeeds.vxMetersPerSecond;
    ySpeed = mSpeeds.vyMetersPerSecond;
    return null;
    //new SwerveControllerCommand(null, null, null, null, null, null)
  }
  /**
   * picks which auto to run
   * 
   * @return our auto to run which is just basically our goal pose
   *         Notes:
   *         value for rotation will need to be adjusted
   *         aswell as pose with testing
   *         this mainly just needs a lot of setting values to work and thats it
   *         just deleted a besic for what they will look like becasue i redid the
   *         trajectory gen and following
   */

  public Command PickAutoToRun() {

    setValues();
    return null;

  }

  private void setValues() {
    if (set == false) {
      goalPose2d = new Pose2d(currentPose2d.getX() + 2, currentPose2d.getY(), Rotation2d.fromDegrees(0));
      ArrayList<Translation2d> nList = new ArrayList<>();
      nList.add(new Translation2d(currentPose2d.getX() + 1, currentPose2d.getY()));
      setWaypoints(nList);
      set = true;
    }
  }

  public void setWaypoints(ArrayList<Translation2d> nArrayList) {
    mInteriorPoints = nArrayList;
  }

  /**
   * 
   * @param c           - the current pose
   * @param g           - the goal pose
   * @param typeToCheck - the type to check - x for x value, y for y value, r for
   *                    rotation
   * @return returns a bool which decides whether or whether not to stop
   */
  private Boolean checking(Pose2d c, Pose2d g, char typeToCheck) {
    if (Character.compare(typeToCheck, 'x') == 0) {

      if (c.getX() > g.getX()) {
        SmartDashboard.putBoolean("type x", true);
        return true;
      }
    } else if (Character.compare(typeToCheck, 'y') == 0) {
      if (Math.abs(c.getY()) > Math.abs(g.getY()) + 0.25) {
        return true;
      }
    } else if (Character.compare(typeToCheck, 'r') == 0) {
      if (Math.abs(c.getRotation().getDegrees()) > Math.abs(g.getRotation().getDegrees()) + 3) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    SmartDashboard.putNumber("goal x", goalPose2d.getX());
    // if (DriverStation.isAutonomous() && autoHasBeenPicked == true) {
    // followAuto(mGenSpeeds(trajectory, ltvController));
    // }
    // SmartDashboard.putNumber("testing",
    // drivetrainAuto.getState().Speeds.vxMetersPerSecond);
  }
}
