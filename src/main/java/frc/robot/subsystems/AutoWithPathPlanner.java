// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoWithPathPlanner extends SubsystemBase {
  /** Creates a new AutoWithPathPlanner. */
  PathPlannerAuto autoPartOne;
  WaitCommand mCommand;
  


  public AutoWithPathPlanner(Intake mIntake,Shooter mShooter) {
  autoPartOne = new PathPlannerAuto("left blue auto v1");
    
    
  }
  public PathPlannerAuto getAuto(int part){
    switch (part) {
      case 1:
        return autoPartOne;
      
      
    }
    return null;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
