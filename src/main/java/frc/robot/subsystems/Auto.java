// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;

public class Auto extends SubsystemBase {
  /** Creates a new Auto. */
  private boolean firstRunOfTraj = false;
  private boolean autoHasBeenPicked;
  private double xForClimb;
  private double yForClimb;
  private Pose2d goalPose2d;
  private Pose2d currentPose2d;
  private SwerveRequest.FieldCentric drive;
  private CommandSwerveDrivetrain drivetrainAuto;
  private Trajectory trajectory;
  private TrajectoryConfig trajectoryConfig;
  private LTVUnicycleController ltvController;
  private ChassisSpeeds chassisSpeeds;
  static double[] xValueForWayPoints = new double[10];
  static double[] yValueForWayPoints = new double[10];

  private double[] rotationValues;
  private boolean firstStage = true;

  public Auto(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric mCentric) {
    this.drivetrainAuto = drivetrain;
    this.drive = mCentric;
    currentPose2d = drivetrainAuto.getState().Pose;

    
    // we can tune this some more when we are at arc field
    // this is for path of the auto from start to where we want to be
    ltvController = new LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0), 0.02, 9);
    
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
    trajectoryConfig = new TrajectoryConfig(9, 5);
    /**
     * might end up having the list of get passed in but for now we are just having
     * the point to go through be the starting point
     */

   
    trajectory = TrajectoryGenerator.generateTrajectory(c,
        List.of(new Translation2d(1,1)), g, trajectoryConfig);

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
    if (firstRunOfTraj) {
      chassisSpeeds = cont.calculate(currentPose2d, goalPose2d, 5, 2);
      return chassisSpeeds;
    }

    chassisSpeeds =  cont.calculate(currentPose2d, goalPose2d, 5, 2);
    firstRunOfTraj = true;
    return chassisSpeeds;
  }

  /**
   * 
   * @param mSpeeds our chassis speeds getting passed into the drivetrain
   */
  private boolean followAuto(ChassisSpeeds mSpeeds) {
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
    if (mSpeeds.omegaRadiansPerSecond == 0 && mSpeeds.vxMetersPerSecond == 0 && mSpeeds.vyMetersPerSecond == 0) {
      
      return false;
    }
     new SequentialCommandGroup(new InstantCommand(
         () -> drivetrainAuto.applyRequest(() -> drive.withVelocityX(mSpeeds.vxMetersPerSecond)))
         .alongWith(
             new InstantCommand(() -> drivetrainAuto.applyRequest(() -> drive.withVelocityY(mSpeeds.vyMetersPerSecond)))
                 .alongWith(new InstantCommand(
                     () -> drivetrainAuto
                         .applyRequest(() -> drive.withRotationalRate(mSpeeds.omegaRadiansPerSecond))))).until(()->currentPose2d.getX() == goalPose2d.getX()));
  
         
    return true;

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

  public InstantCommand PickAutoToRun() {
    double x = currentPose2d.getX();
    double y = currentPose2d.getY();
    if (x > 7) {
      if (y < 3) {

      } else if (y > 5) {

      } else {

      }
    } else {
      if (y < 3) {
        // // Left blue auto
        //goalPose2d = new Pose2d(7.724, 6.457, new Rotation2d(-90));
        goalPose2d = new Pose2d(20,20,new Rotation2d(0));
         while (firstStage == true) {
           firstStage = followAuto(mGenSpeeds(trejGen(currentPose2d, goalPose2d), ltvController));
         }
          xValueForWayPoints[0] = 7.724;
        xValueForWayPoints[1] = 7.724;
        xValueForWayPoints[2] = 7.724;
        xValueForWayPoints[3] = 7.724;
        xValueForWayPoints[4] = 5.704;
        yValueForWayPoints[0] = 6.457;
        yValueForWayPoints[1] = 5.663;
        yValueForWayPoints[2] = 4.715;
        yValueForWayPoints[3] = 3.953;
        yValueForWayPoints[4] = 5.158;
        

        goalPose2d = new Pose2d(2.921, 5.158, new Rotation2d(-90));
      } else if (y > 5) {
        goalPose2d = new Pose2d(7.724, 6.457, new Rotation2d(-90));

        while (firstStage == true) {
          firstStage = followAuto(mGenSpeeds(trejGen(currentPose2d, goalPose2d), ltvController));
        }
        ;
        xValueForWayPoints[0] = 7.724;
        xValueForWayPoints[1] = 7.724;
        xValueForWayPoints[2] = 7.724;
        xValueForWayPoints[3] = 7.724;
        xValueForWayPoints[4] = 5.704;
        yValueForWayPoints[0] = 6.457;
        yValueForWayPoints[1] = 5.663;
        yValueForWayPoints[2] = 4.715;
        yValueForWayPoints[3] = 3.953;
        yValueForWayPoints[4] = 5.158;

        goalPose2d = new Pose2d(2.921, 5.158, new Rotation2d(-90));

      } else {

      }
    }
    autoHasBeenPicked = true;
    
    return new InstantCommand(()->followAuto(mGenSpeeds(trejGen(currentPose2d,goalPose2d), ltvController)));
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
   * 
   * @param x the x coord we want to line up with and we drive towards it will a
   *          speed of 0.5m/s and go till were within 1% of our goal
   * @param y the y coord we want to line up with and we drive towards it will a
   *          speed of 0.5m/s and go till were within 1% of our goal
   *          It does a sequental command group where it drives with velocity
   *          until it is near the goal coords with a tolerance of 1%
   *          42 inches away from x coord
   *          blue 43,147
   *          red 606,170
   */
  private void alignWithClimb() {

    if (DriverStation.getAlliance().get().toString() == "blue") {
      xForClimb = edu.wpi.first.math.util.Units.inchesToMeters(45);
      yForClimb = edu.wpi.first.math.util.Units.inchesToMeters(147);
    } else if (DriverStation.getAlliance().toString() == "red") {
      xForClimb = edu.wpi.first.math.util.Units.inchesToMeters(606);
      yForClimb = edu.wpi.first.math.util.Units.inchesToMeters(170);
    }
    new SequentialCommandGroup(drivetrainAuto.applyRequest(() -> drive.withVelocityX(0.5))
        .until(() -> drivetrainAuto.getState().Pose.getMeasureX().isNear(Units.Meters.of(xForClimb), 0.01))
        .andThen(drivetrainAuto.applyRequest(() -> drive.withVelocityY(0.5))
            .until(() -> drivetrainAuto.getState().Pose.getMeasureY().isNear(Units.Meters.of(yForClimb), 0.01))));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    SmartDashboard.putNumber("x", drivetrainAuto.getState().Pose.getX());

  }
}
