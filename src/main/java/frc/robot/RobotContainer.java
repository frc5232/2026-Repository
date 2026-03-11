// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.Comp1TunerConstatnts;
import frc.robot.generated.Comp1TunerConstatnts;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

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

    // private final Vision visionSubsystem = new Vision(drivetrain);
    // private final Auto autoSubsystem = new Auto(drivetrainSubsystem,drive);
   // private final Aiming aimingSubsystem = new Aiming(drivetrainSubsystem, drive);
   // private final Intake intakeSubsystem = new Intake();;
   // private final Climber climberSubsystem = new Climber();
   // private final Shooter shooterSubsysem = new Shooter();
     //   private final AutoWithPathPlanner autoWithPathPlannerSubsystem = new AutoWithPathPlanner(intakeSubsystem, shooterSubsysem);
    // private final Constants mConstants = new Constants();
   // private final StateMachine stateSubsystem = new StateMachine(intakeSubsystem,shooterSubsysem,drivetrainSubsystem);

   // private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.pov(180)
                .onTrue(drivetrainSubsystem.runOnce(() -> drivetrainSubsystem.resetRotation(new Rotation2d(0))));

        // joystick.pov(0).toggleOnTrue(new
        // InstantCommand(()->mIntake.intakeBalls())).toggleOnFalse(new
        // InstantCommand(()-> mIntake.stopIntake()));
        // joystick.pov(0).onChange(mIntake.increasePositionBy1());
        // joystick.a().onChange(mIntake.increasePositionBy1());
        drivetrainSubsystem.registerTelemetry(logger::telemeterize);

        // joystick.leftBumper().onTrue(shooterSubsysem.shootOutWithVelocity());
         //joystick.pov(0).onTrue(new InstantCommand(()->shooterSubsysem.stopShootingWithVelocity()));
         //joystick.pov(90).onTrue(new InstantCommand(()->intakeSubsystem.goToDownPositionCommand()));
         //joystick.pov(270).onTrue(new InstantCommand(()->intakeSubsystem.gotoStartPositonCommand()));

        // joystick.pov(0).whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.pov(90).whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // joystick.pov(180).whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.pov(270).whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        //        joystick.x().whileTrue(new InstantCommand(()-> intakeSubsystem.moveUpwards()));
        //joystick.y().whileTrue(new InstantCommand(()-> intakeSubsystem.moveDownwards()));

    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(drivetrainSubsystem.applyRequest(()-> drive.withVelocityX(4)).withTimeout(1)).withTimeout(1).andThen(drivetrainSubsystem.applyRequest(()->drive.withVelocityY(-5)).withTimeout(2));
        //return autoChooser.getSelected();
    }

}
