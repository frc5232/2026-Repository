// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterMotorCon;

public class AutoWithPathPlanner extends SubsystemBase {
  /** Creates a new AutoWithPathPlanner. */
  PathPlannerAuto autoPartOne;
  PathPlannerAuto autoPartTwo = new PathPlannerAuto("Left blue part 2");
  PathPlannerAuto autoPartThree = new PathPlannerAuto("Left blue part 3");
 
  AutoBuilder mAutoBuilder = new AutoBuilder();

  public AutoWithPathPlanner(Intake mIntake,Shooter mShooter) {
     NamedCommands.registerCommand("IntakeDown", new InstantCommand(()->mIntake.goToDownPositionCommand()));
    NamedCommands.registerCommand("shoot", mShooter.shootOutWithVelocity());
    this.autoPartOne = new PathPlannerAuto("left blue part 1");
  }
  public PathPlannerAuto getAuto(int part){
    switch (part) {
      case 1:
        return this.autoPartOne;
      case 2:
        return autoPartTwo;
      case 3:
        return autoPartThree;
      
    }
    return null;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
