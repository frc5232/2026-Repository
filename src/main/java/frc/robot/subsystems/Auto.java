
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase {
  /** Creates a new Auto. */
  private boolean firstRunOfTraj = false;
  private boolean autoHasBeenPicked;
  private Pose2d goalPose2d;
  private Pose2d currentPose2d;
  private SwerveRequest.FieldCentric drive;
  private CommandSwerveDrivetrain drivetrainAuto;
  private Trajectory trajectory;
  private TrajectoryConfig trajectoryConfig;
  private LTVUnicycleController ltvController;
  private ChassisSpeeds chassisSpeeds;
  public List<Translation2d> mList;
  public ArrayList<Translation2d> mArrayList = new ArrayList<>();
  public Auto(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric mCentric) {
    drivetrainAuto = drivetrain;
    drive = mCentric;
    currentPose2d = drivetrainAuto.getState().Pose;

    trajectory = new Trajectory();
    // we can tune this some more when we are at arc field
    // this is for path of the auto from start to where we want to be
    ltvController = new LTVUnicycleController(VecBuilder.fill(1, 1, 2.0), VecBuilder.fill(10.0, 20.0), 0.02, 9);
    
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
  private Trajectory trejGen(Pose2d c, Pose2d g, ArrayList<Translation2d> nList) {
    trajectoryConfig = new TrajectoryConfig(100, 100);
    /**
     * might end up having the list of get passed in but for now we are just having
     * the point to go through be the starting point
     */

   
    trajectory = TrajectoryGenerator.generateTrajectory(c, nList, g, trajectoryConfig);   
    
    return trajectory;
  }

  /**
   * 
   * @param tj   our trajectory generated at the start of auto by pick auto to run
   * @param cont our controller as defined in method header
   * @return a chassis speeds which is called in peridoic to update the speeds and
   *         rotations of our drivetrain
   */
  private ChassisSpeeds mGenSpeeds(Trajectory tj, LTVUnicycleController cont) {
    if (this.firstRunOfTraj) {
      chassisSpeeds = cont.calculate(currentPose2d, tj.sample(tj.getTotalTimeSeconds()));
     
      return chassisSpeeds;
    }
    chassisSpeeds = cont.calculate(currentPose2d, tj.sample(0));
    
    this.firstRunOfTraj = true;
    return chassisSpeeds;
  }

  /**
   * 
   * @param mSpeeds our chassis speeds getting passed into the drivetrain
   */
  
  private Command followAuto(ChassisSpeeds mSpeeds) {
    /**
     * Making chassisSpeeds using our controller and how far we are in the
     * trajectory
     * Might want to be updated constantly
     * Make it so traj calc and command group is diffrent methods then when we are
     * close enough to goal pose it just skips over it using if statement
     * 
     */
  
    if (checking(currentPose2d, goalPose2d, 'x')) {
      mSpeeds.vxMetersPerSecond = 0;
    }
    if (checking(currentPose2d, goalPose2d, 'y')) {
      mSpeeds.vyMetersPerSecond = 0;
    }
    if (checking(currentPose2d, goalPose2d, 'r')) {
      mSpeeds.omegaRadiansPerSecond = 0;
    }
    
    return new SequentialCommandGroup(new InstantCommand(
        () -> drivetrainAuto.applyRequest(() -> drive.withVelocityX(mSpeeds.vxMetersPerSecond)))
        .alongWith(
            new InstantCommand(() -> drivetrainAuto.applyRequest(() -> drive.withVelocityY(mSpeeds.vyMetersPerSecond)))
                .alongWith(new InstantCommand(
                    () -> drivetrainAuto.applyRequest(() -> drive.withRotationalRate(mSpeeds.omegaRadiansPerSecond))))
        ));
                    
    

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
    
   // this.mList.add(new Translation2d(3.2,7.3));
    this.goalPose2d = new Pose2d(7.7,6.3, new Rotation2d(90));
    mArrayList.add(new Translation2d(3.2,7.3));
   return followAuto(mGenSpeeds(trejGen(currentPose2d, goalPose2d,mArrayList),ltvController));
    
  
      
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
    if (typeToCheck == 'x') {
      if (c.getX() > g.getX() - 0.25) {
        return true;
      }
    } else if (typeToCheck == 'y') {
      if (c.getY() > g.getY() - 0.25) {
        return true;
      }
    } else if (typeToCheck == 'r') {
      if (c.getRotation().getDegrees() > g.getRotation().getDegrees() - 3) {
        return true;
      }
    }
    return false;
  }

  

  /**
   * Checking our goal pose so that way it wont keep trying to drive towards a
   * goal if its close enough to it
   * mainly for handling and making it easier to use
   * I dont even think its needed but just in case
   */
  private void goalPoseChecking() {
    Pose2d m = drivetrainAuto.getState().Pose;
    double x = m.getX();
    double y = m.getY();

    double dx = goalPose2d.getX();
    double dy = goalPose2d.getY();
    Rotation2d dR = goalPose2d.getRotation();
    if (m.getMeasureX().isNear(goalPose2d.getMeasureX(), 0.03)) {

      goalPose2d = new Pose2d(x, dy, dR);
      dx = goalPose2d.getX();
      dy = goalPose2d.getY();
      dR = goalPose2d.getRotation();
    } else if (m.getMeasureY().isNear(goalPose2d.getMeasureY(), 0.03)) {
      goalPose2d = new Pose2d(dx, y, dR);
      dx = goalPose2d.getX();
      dy = goalPose2d.getY();
      dR = goalPose2d.getRotation();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    if (DriverStation.isAutonomous() && autoHasBeenPicked == true) {
      followAuto(mGenSpeeds(trajectory, ltvController));
    }

  }
}

