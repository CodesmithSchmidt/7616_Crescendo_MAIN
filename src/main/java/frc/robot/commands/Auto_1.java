// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm2;
import frc.robot.subsystems.Scoring;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Auto_1 extends Command {
  /** Creates a new Auto_1. */

  private DriveSubsystem a_Swerve = new DriveSubsystem();
private Arm2 a_arm2 = new Arm2();
private Scoring a_scoring = new Scoring();

  private Timer a_timer = new Timer();

  public Auto_1(DriveSubsystem a_swerve, Arm2 a_arm, Scoring a_scoring) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.a_Swerve = a_Swerve;
    this.a_arm2 = a_arm2;
    this.a_scoring = a_scoring;

    addRequirements(a_arm2);
    addRequirements(a_scoring);
    addRequirements(a_Swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    a_timer.reset();
    a_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        a_Swerve::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        a_Swerve::setModuleStates,
        a_Swerve);

    // Reset odometry to the starting pose of the trajectory.
    a_Swerve.resetOdometry(exampleTrajectory.getInitialPose());

    a_arm2.setPosition(175);
    while(a_timer.get() < 2){
      a_scoring.shooting();
    }
    while(a_timer.get() < 4){
      a_scoring.shooting();
      a_scoring.intake();
    }
    a_scoring.stopShooting();
    a_scoring.intakeStop();

    swerveControllerCommand.andThen(() -> a_Swerve.drive(0, 0, 0, true, false));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

//    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, false));








/*


package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Swerve;

public class timeAutonomous extends CommandBase {
  /** Creates a new timeAutonomous. 

  private Swerve a_Swerve = new Swerve();
  private Timer a_timer = new Timer();

  public timeAutonomous(Swerve a_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.a_Swerve = a_Swerve;

    addRequirements(a_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    a_timer.reset();
    a_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    //For 2 seconds drive forward at 25% speed
    while(a_timer.get() < 2){
            a_Swerve.drive(new Translation2d(.65,0), 0, true, false);
      

    }

    //While gyro is not level, drive the correct direction to level
    while(Math.abs(a_Swerve.gyro.getPitch()) > 10){
      if(a_Swerve.gyro.getPitch() > 0){
        a_Swerve.drive(new Translation2d(.5,0),0,true,false);
      }
      else {
        a_Swerve.drive(new Translation2d(-.5,0),0,true,false);
      }
    }
    
    a_Swerve.drive(new Translation2d(0,0),0,true,false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    a_Swerve.drive(new Translation2d(0,0),0,true,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    a_Swerve.drive(new Translation2d(0,0),0,true,false);
    return false;
  }
}
 */