/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;

import static edu.wpi.first.wpilibj.util.Units.inchesToMeters;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class   RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();

  ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

      //voltage constant to ensure we don't accelerate too fast

      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(Constants.ksVolts,
                                         Constants.kvVoltSecondsPerMeter,
                                         Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, 10);


      //create config for trajectory

      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1, 2)
            //kinematics to ensure max speed is obeyed + applying voltage constraint
      .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

      TrajectoryConfig complexTrajectoryConfig = new TrajectoryConfig(1, 2)
              //kinematics to ensure max speed is obeyed + applying voltage constraint
      .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

      //simple trajectory to follow

      Rotation2d rotation = new Rotation2d(Math.PI / 4);
      Pose2d start = new Pose2d(4.0, 5.0, rotation);
      Pose2d end = new Pose2d(8.0, 3.0, rotation);
      Translation2d trans1 = new Translation2d(5, 4.5);
      Translation2d trans2 = new Translation2d(6, 4);
      Translation2d trans3 = new Translation2d(7, 3.5);
      ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(trans1);
      interiorWaypoints.add(trans2);
      interiorWaypoints.add(trans3);

      Trajectory simpleTrajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, trajectoryConfig);

      //complex trajectory to follow

      Rotation2d complexRotation = new Rotation2d(Math.PI/2);
      Pose2d complexStart = new Pose2d(inchesToMeters(265.33), inchesToMeters(154.875), complexRotation);
      Pose2d complexEnd = new Pose2d(inchesToMeters(332.24), inchesToMeters(349.505), complexRotation);
      Translation2d point1 = new Translation2d(inchesToMeters(282.058), inchesToMeters(189.527));
      Translation2d point2 = new Translation2d(inchesToMeters(315.513), inchesToMeters(206.853));
      Translation2d point3 = new Translation2d(inchesToMeters(332.24), inchesToMeters(241.505));
      Translation2d point4 = new Translation2d(inchesToMeters(332.24), inchesToMeters(349.505));
      ArrayList <Translation2d> complexInteriorWaypoints = new ArrayList<Translation2d>();
      complexInteriorWaypoints.add(point1);
      complexInteriorWaypoints.add(point2);
      complexInteriorWaypoints.add(point3);
      complexInteriorWaypoints.add(point4);

      Trajectory complexTrajectory = TrajectoryGenerator.generateTrajectory(complexStart, complexInteriorWaypoints, complexEnd, complexTrajectoryConfig);


      RamseteCommand simpleRamseteCommand = new RamseteCommand(
              simpleTrajectory, m_driveTrain::getPose,
              new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
              new SimpleMotorFeedforward(Constants.ksVolts,
                                         Constants.kvVoltSecondsPerMeter,
                                         Constants.kaVoltSecondsSquaredPerMeter),
              Constants.kDriveKinematics,
              m_driveTrain::getWheelSpeeds,
              new PIDController(Constants.kPDriveVel, 0, 0),
              new PIDController(Constants.kPDriveVel, 0, 0),
              //Ramsete command passes volts to the callback
              m_driveTrain::tankDriveVolts,
              m_driveTrain
      );


      // run path following command, then stop at the end

      //if simple, use return statement below
      return simpleRamseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0,0));

      //if complex, use return statement below
      //return complexRamseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0,0));


  }
}
