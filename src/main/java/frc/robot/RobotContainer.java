// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  @Logged(name = "drivetrain")
  private final DrivetrainSubsystem mDrivetrain = new DrivetrainSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController mDriverController = new CommandXboxController(0);

  private final Vision mVision = new Vision();

  SwerveInputStream mDriveFieldOriented = SwerveInputStream.of( //
      mDrivetrain.getSwerveDrive(), //
      () -> mDriverController.getLeftY() * -1, //
      () -> -mDriverController.getLeftX()) //
      .withControllerRotationAxis(() -> -mDriverController.getRightX()).deadband(0.1)
      .scaleTranslation(0.8).allianceRelativeControl(false);

  SwerveInputStream mDriveRobotOriented = mDriveFieldOriented.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream mSlowerDriveFieldOriented = SwerveInputStream.of( //
      mDrivetrain.getSwerveDrive(), //
      () -> mDriverController.getLeftY() * 0.25 * -1, //
      () -> mDriverController.getLeftX() * 0.25)
      .withControllerRotationAxis(() -> -mDriverController.getRightX()).deadband(0.1)
      .scaleTranslation(1.0).allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mDrivetrain.setDefaultCommand(mDrivetrain.driveFieldOriented(mDriveFieldOriented));
    mDriverController.a().onTrue(mDrivetrain.resetGyroCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
