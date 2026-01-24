// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase {
  /** Creates a new Auto. */
  private Pose3d startingPose3d;
  private Pose3d goalPose;
  private Pose3d currentPose3d;
  public Auto() {}

  public boolean checkAccuracy(){
    if (goalPose.getX() - currentPose3d.getX() <= 0.1){
      return true;
    }else{
      return false;
    }
  }
  public boolean isFinished() {
    if (checkAccuracy() == true){
      return true;}
      else{
        return false;
      }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
