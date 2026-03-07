// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.derive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autov2 extends SubsystemBase {
  /** Creates a new Autov2. */
    private final SendableChooser<Command> m_autoChooser;


    // --- Auto Chooser ---

    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("None", null);

    // Add autos to chooser
    m_autoChooser.addOption("left", AutoBuilder.buildAuto("left"));
    m_autoChooser.addOption("right", AutoBuilder.buildAuto("right"));


    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
