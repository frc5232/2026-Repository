package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aiming extends SubsystemBase {
  /** Creates a new Aiming. */
  CommandSwerveDrivetrain drivetrainAiming;
  FieldCentric drive;
  Pose2d currentPose;
  Pose2d staticHubPose2d;
  double yawHeading;
  public boolean aim = false;

  public Aiming(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric mCentric) {
    /**
     * Change to actual position of hub pose
     * current pose will just be drivetrain.getstate.pose.get(value)
     */
    drivetrainAiming = drivetrain;
    pickHubPose();
    currentPose = drivetrainAiming.getState().Pose;
    drive = mCentric;

  }

  private void calculate() {
    if (aim) {
      // doubls is our pose - hub pose
      double x = Math.abs(currentPose.getX() - staticHubPose2d.getX());
      double y = Math.abs(currentPose.getY() - staticHubPose2d.getY());
      // doing a angle calulation
      double xx = (x * Math.sin(90)) / Math.sqrt((x * x + y * y));

      //tsting theta idea from herstad in discord
      yawHeading = currentPose.getRotation().getDegrees() + xx;
      // for blue alliance
      if (currentPose.getX() > 7) {
        if (currentPose.getY() < 4) {
          // for away from scoring table
          yawHeading = xx;
        } // close to scoring table
        else {
          yawHeading = 360 - xx;
        }

      } // for our red alliance
      else if (currentPose.getX() < 7) {
        // if closer to scoring table
        if (currentPose.getY() < 4) {
          yawHeading = 180 - xx;
        } // if away from scoring table
        else {
          yawHeading = 180 + xx;
        }
      }
     if(Math.abs(currentPose.getX() - staticHubPose2d.getX()) < 5){
      new SequentialCommandGroup(drivetrainAiming.applyRequest(() -> drive.withRotationalRate(4))
          .until(() -> yawheadChecker(drivetrainAiming.getState().RawHeading.getDegrees(), yawHeading)));
     }
    }

    // then constantly update it so we can drive and aim auto while shooting until
    // disabled
  }

  /**
   * hook up to a button so we can pick if we want to aim
   */
  private void changeAimState() {
    aim = !aim;
  }
  /**
   * sets the value of the hub pose based off of what alliance we are in
   */
  private void pickHubPose() {
    if (DriverStation.getAlliance().get().toString() == "red") {
      staticHubPose2d = new Pose2d(Units.inchesToMeters(469.11), Units.inchesToMeters(159.1), new Rotation2d(0));
    } else {
      staticHubPose2d = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(159.1), new Rotation2d(0));
    }
  }
  /**
   * 
   * @param currentYaw our current drivetrain yaw / what every yaw w want to say were currentlyu at
   * @param goalYaw our goal yaw
   * @return true or false, true if near it
   */
  private boolean yawheadChecker(double currentYaw, double goalYaw) {
    if (currentYaw + 2 >= goalYaw || currentYaw - 2 <= goalYaw) {
      return true;
    }
    return false;
  }
  /**
   * updates our current pose
   */
  private void updateCurrentPose() {
    currentPose = drivetrainAiming.getState().Pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculate();
    updateCurrentPose();
  }
}
