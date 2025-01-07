package frc.lib;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.configs.*;
import frc.robot.classes.TunableValue;
import org.photonvision.PhotonPoseEstimator;

public final class Constants {
    public static final boolean DEBUG = false;
    public static final double stickDeadband = 0.1;

    public static final driveTrainConfig drivetrainconfig =  new driveTrainConfig(Units.inchesToMeters(20.75), Units.inchesToMeters(20.75), 2);
    public static final RobotConfig robotConfig = new RobotConfig(10, 10, new ModuleConfig(2, 10, 1.0, new DCMotor(12, 0.5, 60, 0.5, 6000, 1), 40, 1), new Translation2d(drivetrainconfig.wheelBase / 2.0, drivetrainconfig.trackWidth / 2.0), new Translation2d(drivetrainconfig.wheelBase / 2.0, -drivetrainconfig.trackWidth / 2.0), new Translation2d(-drivetrainconfig.wheelBase / 2.0, drivetrainconfig.trackWidth / 2.0), new Translation2d(-drivetrainconfig.wheelBase / 2.0, -drivetrainconfig.trackWidth / 2.0));

    public static final class SwerveConstants {
        public static final COTSswerveconfig chosenModule = COTSswerveconfig.KrakenX60Falcon500MK4i(COTSswerveconfig.driveRatios.L3);
        public static final swervemoduleconfig mod0 = new swervemoduleconfig(12, 13, 11, Rotation2d.fromRotations(0.299805)); // Front Left Module
        public static final swervemoduleconfig mod1 = new swervemoduleconfig(22, 23, 21, Rotation2d.fromRotations(0.343018)); // Front Right Module
        public static final swervemoduleconfig mod2 = new swervemoduleconfig(32, 33, 31, Rotation2d.fromRotations(0.128906)); // Back Left Module
        public static final swervemoduleconfig mod3 = new swervemoduleconfig(42, 43, 41, Rotation2d.fromRotations(0.060303)); // Back Right Module
        public static final driveTrainConfig drivetrainconfig =  new driveTrainConfig(Units.inchesToMeters(20.75), Units.inchesToMeters(20.75), chosenModule.wheelCircumference);
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(new Translation2d(drivetrainconfig.wheelBase / 2.0, drivetrainconfig.trackWidth / 2.0), new Translation2d(drivetrainconfig.wheelBase / 2.0, -drivetrainconfig.trackWidth / 2.0), new Translation2d(-drivetrainconfig.wheelBase / 2.0, drivetrainconfig.trackWidth / 2.0), new Translation2d(-drivetrainconfig.wheelBase / 2.0, -drivetrainconfig.trackWidth / 2.0));
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double maxSpeed = 3.93;
        public static final double maxAngularVelocity = 10.0;
        public static final PIDConfig anglemotorPID = new PIDConfig(40, 0.4, 2.5);
        public static final PIDConfig drivemotorPID = new PIDConfig(1.25, 0.01, 0.1125);
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;
        public static final PIDConfig pathplannertranslationpid = new PIDConfig(1, 0, 0);
        public static final PIDConfig pathplannerrotationpid = new PIDConfig(1, 0, 0);
    }


    public static final int pigeonID = 4;
    

    

    public static final cameraConfig camera1Config = new cameraConfig(
            "camera1",
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            );

    


}
