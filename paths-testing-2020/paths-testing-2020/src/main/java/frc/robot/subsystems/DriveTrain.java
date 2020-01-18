/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private static WPI_TalonSRX rightTalon1 = new WPI_TalonSRX(4);
  private static WPI_TalonSRX rightTalon2 = new WPI_TalonSRX(5);
  private static WPI_TalonSRX rightTalon3 = new WPI_TalonSRX(6);
  private static WPI_TalonSRX rightTalon4 = new WPI_TalonSRX(7);

  private static WPI_TalonSRX leftTalon1 = new WPI_TalonSRX(12);
  private static WPI_TalonSRX leftTalon2 = new WPI_TalonSRX(13);
  private static WPI_TalonSRX leftTalon3 = new WPI_TalonSRX(14);
  private static WPI_TalonSRX leftTalon4 = new WPI_TalonSRX(15);

  public static WPI_TalonSRX gyroTalon = new WPI_TalonSRX(1);

  public static Encoder driveLeftEncoder = new Encoder(0, 1);
  public static Encoder driveRightEncoder = new Encoder(2, 3);

  private SpeedControllerGroup rightTalons = new SpeedControllerGroup(rightTalon1, rightTalon2, rightTalon3, rightTalon4);
  private SpeedControllerGroup leftTalons = new SpeedControllerGroup(leftTalon1, leftTalon2, leftTalon3, leftTalon4);


  public static PigeonIMU gyro = new PigeonIMU(gyroTalon);

  Rotation2d initialPosition = new Rotation2d(gyro.getCompassHeading());
  final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(initialPosition);

  private final DifferentialDrive driveBase = new DifferentialDrive(leftTalons, rightTalons);
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);

  public DriveTrain() {


  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(driveLeftEncoder.getRate(), driveRightEncoder.getRate());
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getCompassHeading(), 360);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftTalons.setVoltage(leftVolts);
    rightTalons.setVoltage(-rightVolts);
  }

  public void resetEncoders() {
    driveLeftEncoder.reset();
    driveRightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return ((driveLeftEncoder.getDistance() + driveRightEncoder.getDistance()) / 2.0);
  }

  public Encoder getLeftEncoder() {
    return driveLeftEncoder;
  }

  public Encoder getRightEncoder() {
    return driveRightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    driveBase.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.setCompassAngle(0);
  }

  public double getTurnRate() {

    return 8.0;
    //return gyro.get
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
