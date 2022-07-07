// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kOI;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController xboxController = new XboxController(kOI.DRIVE_CONTROLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(new RunCommand(()-> drivetrain.curveDrive(OI.getRightAxis(xboxController), OI.getLeftAxis(xboxController), xboxController.getXButton()), drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(kDrivetrain.ksVolts,
                                 kDrivetrain.kvVoltSecondsPerMeter,
                                 kDrivetrain.kaVoltSecondsSquaredPerMeter),
      kDrivetrain.kDriveKinematics,
      10
    );

    TrajectoryConfig config = new TrajectoryConfig(
      kDrivetrain.kMaxSpeedMetersPerSecond,
      kDrivetrain.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(kDrivetrain.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0),
            new Translation2d(1, 0)
        ),
        new Pose2d(2, 0, new Rotation2d(0)),
        config
    );

    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrain::getPose,
        new RamseteController(kDrivetrain.kRamseteB, kDrivetrain.kRamseteZeta),
        new SimpleMotorFeedforward(kDrivetrain.ksVolts,
                                   kDrivetrain.kvVoltSecondsPerMeter,
                                   kDrivetrain.kaVoltSecondsSquaredPerMeter
        ),
        kDrivetrain.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(kDrivetrain.kPDriveVel, 0, 0),
        new PIDController(kDrivetrain.kPDriveVel, 0, 0),
        drivetrain::tankDriveVolts,
        drivetrain
    );

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}
