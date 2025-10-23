package frc.robot.subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

@Logged
public class DrivetrainSubsystem extends SubsystemBase {
  public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(7.5);

  private static final boolean PATHPLANNER_ENABLE_FEEDFORWARD = true;

  @NotLogged
  private final SwerveDrive mSwerveDrive;

  public DrivetrainSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      var configDir = new File(Filesystem.getDeployDirectory(), "swerve");
      var parser = new SwerveParser(configDir);

      mSwerveDrive = parser.createSwerveDrive(MAX_SPEED.in(MetersPerSecond));
    } catch (IOException err) {
      // *We* know that the config will always be deployed to the robot,
      // but Java doesn't. Just bubble the exception up.
      throw new RuntimeException(err);
    }

    mSwerveDrive.setHeadingCorrection(false);
    mSwerveDrive.setCosineCompensator(true);
    mSwerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    mSwerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    initPathPlanner();
    // This is meant to flip the direction of the front of the robot on the alliance
    // station we are at in competition. FIXME Enable only if facing this issue
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
  }

  @NotLogged
  public SwerveDrive getSwerveDrive() {
    return mSwerveDrive;
  }

  @NotLogged
  public SwerveDriveConfiguration getSwerveConfig() {
    return mSwerveDrive.swerveDriveConfiguration;
  }

  @NotLogged
  public SwerveController getSwerveController() {
    return mSwerveDrive.getSwerveController();
  }

  @NotLogged
  public SwerveDriveKinematics getCurrentKinematics() {
    return mSwerveDrive.kinematics;
  }

  public Pose2d getCurrentPose() {
    return mSwerveDrive.getPose();
  }

  public Rotation2d getHeading() {
    return getCurrentPose().getRotation();
  }

  public void resetOdometry(Pose2d pNewPose) {
    mSwerveDrive.resetOdometry(pNewPose);
  }

  public ChassisSpeeds getRobotVelocity() {
    return mSwerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getFieldVelocity() {
    return mSwerveDrive.getFieldVelocity();
  }

  public SwerveModuleState[] getCurrentModuleStates() {
    return mSwerveDrive.getStates();
  }

  public Pose2d getPose() {
    return mSwerveDrive.getPose();
  }

  public ChassisSpeeds convertOperatorInputToChassisSpeeds(double pTranslationX, double pTranslationY,
      Rotation2d pRotation) {
    var scaled = SwerveMath.cubeTranslation(new Translation2d(pTranslationX, pTranslationY));

    return getSwerveController().getTargetSpeeds(scaled.getX(), scaled.getY(), pRotation.getRadians(),
        getHeading().getRadians(), MAX_SPEED.in(MetersPerSecond));
  }

  public ChassisSpeeds convertOperatorInputToChassisSpeeds(double pTranslationX, double pTranslationY,
      double pRotateX,
      double pRotateY) {
    var scaled = SwerveMath.cubeTranslation(new Translation2d(pTranslationX, pTranslationY));

    return getSwerveController().getTargetSpeeds(scaled.getX(), scaled.getY(), pRotateX, pRotateY,
        getHeading().getRadians(), MAX_SPEED.in(MetersPerSecond));
  }

  public Command driveCommand(DoubleSupplier pTranslationX, DoubleSupplier pTranslationY,
      DoubleSupplier pAngularRotation) {
    return run(() -> {
      var translation = new Translation2d(pTranslationX.getAsDouble(), pTranslationY.getAsDouble())
          .times(getSwerveDrive().getMaximumChassisVelocity());
      var scaled = SwerveMath.scaleTranslation(translation, 0.8);

      var rotation = Math.pow(pAngularRotation.getAsDouble(), 3)
          * getSwerveDrive().getMaximumChassisAngularVelocity();

      mSwerveDrive.drive(scaled, rotation, true, false);
    });
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> pSpeedsSupplier) {
    return run(() -> {
      mSwerveDrive.driveFieldOriented(pSpeedsSupplier.get());
    });
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> pSpeedsSupplier,
      DoubleSupplier pTranslationScalarSupplier) {
    return run(() -> {
      var speeds = pSpeedsSupplier.get();
      var scalar = pTranslationScalarSupplier.getAsDouble();

      speeds = speeds.times(scalar);

      mSwerveDrive.driveFieldOriented(speeds);
    });
  }

  public Command resetGyroCommand() {
    return run(this::zeroGyroWithAlliance);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    mSwerveDrive.driveFieldOriented(velocity);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(mSwerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          mSwerveDrive.getOdometryHeading().getRadians(),
          mSwerveDrive.getMaximumChassisVelocity()));
    });
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> pSpeeds) {
    return run(() -> {
      mSwerveDrive.drive(pSpeeds.get());
    });
  }

  @Override
  public void periodic() {
  }

  private void consumePathPlannerOutput(ChassisSpeeds pRobotRelativeSpeeds, DriveFeedforwards pFeedforwards) {
    if (PATHPLANNER_ENABLE_FEEDFORWARD) {
      mSwerveDrive.drive(pRobotRelativeSpeeds, getCurrentKinematics().toSwerveModuleStates(pRobotRelativeSpeeds),
          pFeedforwards.linearForces());
    } else {
      mSwerveDrive.setChassisSpeeds(pRobotRelativeSpeeds);
    }
  }

  private boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    return alliance.map(a -> a.equals(Alliance.Red)).orElse(false);
  }

  private void initPathPlanner() {
    try {
      RobotConfig robotCfg = RobotConfig.fromGUISettings();

      var pathFollower = new PPHolonomicDriveController(Constants.PATH_FOLLOWING_PID_CONSTANTS_POSITIONAL,
          Constants.PATH_FOLLOWING_PID_CONSTANTS_ROTATIONAL);

      AutoBuilder.configure(mSwerveDrive::getPose, mSwerveDrive::resetOdometry, this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (PATHPLANNER_ENABLE_FEEDFORWARD)
              consumePathPlannerOutput(speedsRobotRelative, moduleFeedForwards);
          }, pathFollower,
          robotCfg, this::shouldFlip, this);
    } catch (Exception err) {
      throw new RuntimeException(err);
    }

    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing
   * forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyro() {
    mSwerveDrive.zeroGyro();
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

}
