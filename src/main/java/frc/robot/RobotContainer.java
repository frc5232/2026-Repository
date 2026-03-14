// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.Comp1TunerConstatnts;
import frc.robot.subsystems.Aiming;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooting;
public class RobotContainer {
    private double MaxSpeed = 1.0 * Comp1TunerConstatnts.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrainSubsystem = Comp1TunerConstatnts.createDrivetrain();

    private final Vision visionSubsystem = new Vision(drivetrainSubsystem);
     private final Auto autoSubsystem = new Auto(drivetrainSubsystem,drive);
   private final Aiming aimingSubsystem = new Aiming(drivetrainSubsystem, drive);
  // private final Intake intakeSubsystem = new Intake();;

   private final Shooting shooterSubsystem = new Shooting();
     

    public RobotContainer() {
//        intakeSubsystem = new Intake();
//        climberSubsystem = new Climber();
//        shooterSubsysem = new Shooter();
         //NamedCommands.registerCommand("shoot", shooterSubsysem.shootOutWithVelocity());
        // SmartDashboard.putBoolean("testingpleasework", NamedCommands.hasCommand("shoot"));
        // NamedCommands.registerCommand("IntakeDown",intakeSubsystem.goToDownPositionCommand());
        configureBindings();

        // autoChooser.setDefaultOption("Left", AutoBuilder.buildAuto("left"));
        // autoChooser.addOption("Right", AutoBuilder.buildAuto("right"));

        // SmartDashboard.putData("Auto Chooser", autoChooser);
        // SignalLogger.setPath("/media/sda1/");
       //SignalLogger.start();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrainSubsystem.setDefaultCommand(
                // Drivetrain will execute this command periodically

                drivetrainSubsystem.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)//xAxisRateLimiter.calculate(-joystick.getLeftY()) * MaxSpeed) // Drive forward
                                                                                                    // with negative Y
                                                                                                    // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)//yAxisRateLimiter.calculate(-joystick.getLeftX()) * MaxSpeed) // Drive left with
                                                                                                    // negative X (left)
                        .withRotationalRate(joystick.getRightX() * MaxAngularRate)//rotationRateLimiter.calculate(-joystick.getRightX()) * MaxAngularRate) // Drive
                                                                                                                  // counterclockwise
                                                                                                                  // with
                                                                                                                  // NOT
                                                                                                                  // NEGATIVE!
                                                                                                                  // POSITIVE
                                                                                                                  // X
                                                                                                                  // (right)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrainSubsystem.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrainSubsystem.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrainSubsystem.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        

        // Reset the field-centric heading on left bumper press.
        joystick.pov(180)
                .onTrue(drivetrainSubsystem.runOnce(() -> drivetrainSubsystem.resetRotation(new Rotation2d(0))));

        
        drivetrainSubsystem.registerTelemetry(logger::telemeterize);
      //  joystick.pov(0).onTrue(intakeSubsystem.intakeDownCommand());
        //joystick.pov(90).onTrue(intakeSubsystem.intakeUpCommand());
    }

    public Command getAutonomousCommand() {
        //return new SequentialCommandGroup(drivetrainSubsystem.applyRequest(()-> drive.withVelocityX(4)).withTimeout(1)).withTimeout(1).andThen(drivetrainSubsystem.applyRequest(()->drive.withVelocityY(-5)).withTimeout(2));
        return autoSubsystem.PickAutoToRun();
      // return null;
    }

}
