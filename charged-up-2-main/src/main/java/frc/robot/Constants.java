package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;


public final class Constants {
  
 
    public static final class LEDs {
    /*Led Constants */
      public static final int kPwmChannel = 0;
      public static final int kCandelChannel = 31;
      public static final int kLEDTargetLength = 60; //60
      public static final int kLEDModeLength = 140; //60
      public static final int kLEDBufferLength = 200; // 120
      public static final int kPurple = 215;
      public static final int kGreen = 60;
      public static final int kBlue = 125;
      public static final int kYellow = 10;
      public static final int kWhite = 255;
      public static final int kRed = 0;
      public static final int kPink = 351;
      public static final int kSat0 = 0;
      public static final int kSatDefault = 255;
      public static final int kConeModeColor = 10;
      public static final int kCubeModeColor = 125;
  }

  public static final class RobotMode {
    /* Pipeline  Constants */
      public static final int kCone = 0;
      public static final int kCube = 1;
   
  }

  public static final class ChargingStationConstants {
    //The degree that the robot tolerates as "balanced" on the charging station
    public static final double kAngleVariance = 1.5;
  }

  public static final class Servos {
    
      public static final int kServoChannel = 3;     // not used at this time
  }
  public static final class Intake {
     public static final int kIntakeMotorBottom = 17;
     public static final int kIntakeMotorTop = 14;
     public static final double kIntakeMotorPercentage = 0.5;
     public static final double kIntakeMotorShootMidPercentage = .35;
     public static final double kIntakeMotorShootHighPercentage = 2;
     public static final double kIntakeMotorShootLowPercentage = 0.1;
     public static final int kIntakeSensorPort = 9;
     public static final double kCurrentOutputMax = 6.5;
     public static final double kCycleAmountInSeconds = .5;
     public static final double kCyclePerSecond = 50;
  }

  public static final class PneumaticConstants {
    
      public static final int kIntakeSolenoid = 2;     
      public static final int kClawSolenoid = 1;
      public static final int kWristSolenoid = 0;
      public static final int kPHID = 30;
      public static final double kMinPressure = 90;
      public static final double kMaxPressure = 120.0;
      public static final int kPHChannel = 30;
  }

  public static final class ShoulderConstants {

   
    //CAN IDs
    public static final int kShoulderSpeedControllerCanID = 5; 
    public static final int kExtenderSpeedControllerCanID = 6; 
    public static final int kShoulderCANCoderCanID = 29; 
    
    public static final boolean kShoulderInvert = true; // / inverted so that number go up from back of bot to the front of the bot (limit switch are also supporting this configuration)
    public static final IdleMode kShoulderNeutralMode = IdleMode.kBrake;

    //Setup shoulder motor PIDs
    public static final double kShoulder_P = 0.7;  
    public static final double kShoulder_I = 0;  
    public static final double kShoulder_D = 0.15;  
    public static final double kShoulder_Zone = 0;  
    public static final double kShoulder_Gains = 0;  
    public static final double kShoulder_FF = 0;



    /* motor Current Limiting */
    public static final int kAngleContinuousCurrentLimit = 20;

    /* motor speed limiting */
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  

    // motor  Voltage Compensation */
    public static final double kShoulderVoltageCom = 12.0;
    
    // angle conversion factors
    public static final double kShoulderAngleGearRatio = (100 / 1.0); // 100:1 
    public static final double kShouldAngleConversionFactor = 360.0 / kShoulderAngleGearRatio;


    // Smart Motion Constants
    public static final double kFullRotationTimeSec = 4.0;
    public static final double kMaxVelRPM = (kShoulderAngleGearRatio / kFullRotationTimeSec) * 40; //~350
    public static final double kMinVelRPM = 0.0;
    public static final double kMaxAccel = kMaxVelRPM / 2.0;
    public static final double kAllowedError = 0.0;
    public static final int kSmartMotionSlot = 0;


    //Offset from grid to allow clearance for shoulder raising
    public static final double kGridOffset = 0.08; //6 inches (was 0.1524)
    public static final double kGridReturnOffset = 0.08; // 3.937 inches (accounts for gear slack)
     // Values for cone pick up
     //X value for when robot is engaged to portal ready to pickup item
     public static final double kXEngagedToPortal = 14.0335;
     //X value for when robot is near portal to allow arm clearance
     public static final double kXNotEngagedToPortal = 13.8811;
     //Y distance between AprilTag alignment and item pickup location
     public static final double kYOffsetFromAprilTag = 0.635;

    //Shoulder positions offsets
    public static final double kShoulderKey = 102.393; // this is the offset of the cancoder
    public static final double kShoulderStowPosition = -4 ;  // degrees
    public static final double kShoulderHybridPosition =  35;   // degrees
    public static final double kShoulderConeMiddlePosition =  94;   // degrees
    public static final double kShoulderConeTopPosition = 106;   // degrees
    public static final double kShoulderCubeMiddlePlacePosition = 84;  // degrees
    public static final double kShoulderCubeTopPlacePosition =  100;  // degrees
    public static final double kShoulderPortalPosition =  271;  // degrees
    public static final double kShoulderSweepPosition = -9;
    
 
    
    //Set shoulder motor soft limits
    //positions that are the start and end of the robot guts (do not travel here) assume reverse installation of motor 
    public static final float kShoulder_FwdSoftLimit = 271f; // degrees
    public static final float kShoulder_RevSoftLimit = -10f ; // degrees equivalent to 139 (as the motor moves in reverse)



  }

  public static final class ExtenderConstants {
    
    public static final boolean kExtenderInvert = false;
    public static final IdleMode kExtenderNeutralMode = IdleMode.kCoast;
    
    //Setup extender motor PID
    public static final double kExtender_P = .2;  
    public static final double kExtender_I = 0;  
    public static final double kExtender_D = 0;  
    public static final double kExtender_Zone = 0;  
    public static final double kExtender_Gains = 0;  
    public static final double kExtender_FF = 0;

    //Set max & min outputs
    public static final double kExtender_MinOutput = -1; 
    public static final double kExtender_MaxOutput = 1;  

    //Set extender motor soft limits 
    public static final float kExtender_FwdSoftLimit = 95;  // degrees
    public static final float kExtender_RevSoftLimit = 0;  

    //Extender positions
    public static final double kExtenderRetractedPosition = 0; 
    public static final double kExtenderMiddlePosition = 2.5; // in
    public static final double kExtenderTopPosition = 16;  // in 
    public static final double kExtenderPortalPosition = 0; // in   

    //gear ratio
    public static final double kExtenderGearRatio = (2 / 1.0); // 3:1 

  }

  public static final class LimeLightConstants {
    public static final double kAlignThreshold = 0.04;
    public static final double kRotateThreshold = 1;
    public static final double kCameraHeightH1 = 0.84;
    public static final double kLowTargetHeightH2 = 0; //meters
    public static final double kLowTargetGridDistance = 0.57785; //meters
    public static final double kHighTargetHeightH2 = 1.12; //meters
    public static final double kHighTargetGridDistance = 1.00965; //meters
    public static final double kAprilTargetHeight = 0.46355; //meters
    public static final double kAprilTargetOffset = 0.4572;
    public static final double kCameraAngle = 0;
    public static final double kCameraOffsetDistance = 0.089;
    public static final double kGridCameraDepth = 0.1016; //meters
    public static final double kReflectiveTapePipeline = 0;
    public static final double kAprilTagPipeline = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kMaxSpeedMetersPerSecondGoOnScale = 3.3;
    public static final double kMaxAccelerationMetersPerSecondSquaredGoOnScale = 1.6;
    public static final double kMaxSpeedMetersPerSecondAutonomous = 3;//4
    public static final double kMaxAccelerationMetersPerSecondSquaredAutonomous = 1.45;//1.8

    //public static final String kAuto1 = "Auto1";
    //public static final String kAuto2 = "Auto2";
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  
  
  
  public static final class Swerve {
    public static final double stickDeadband = 0.1; 

    public static final int pigeonID = 28;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

     
        /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(17.5);
    public static final double wheelBase = Units.inchesToMeters(17.5);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1 L2 (this is what we (2377) have)
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 35;// this made the robot smooth like ice

    /* Angle Motor PID Values */
    public static final double angleKP = .03;// up via tuning
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.5;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxQuarterSpeed = 1.25; //meters per second
    public static final double autoBalanceMaxSpeed = .8; // meters per second
    public static final double gridMaxSpeed = 3; //meters per second 
    public static final double autoAlignMaxSpeed = 1;
    public static final double rotateToAngleMaxSpeed = .01; // meters per second (was .006)
    public static final double rotateToAngleMinSpeed = .001;
    public static final double maxAngularVelocity = 11.5;
    public static final double maxQuarterAngularVelocity = 3;
    public static final double rotateAtSpeed = .006;
    public static final double rotateAtSpeedPov= 4.5;//meters per second
    public static final double rotateSpeed = .1; // meters per second for the rotate left and rotate right command
    
    public static final double kPovMovementSpeed = .3; //This will be the speed at which the dpad movement will be at
    public static final double kPovMovementDistance = 1; // in theory this should make it move only 3 inches
    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Slew Limiter */
    public static final double ktranslationLimiter = 1.0; 
    public static final double kstrafeLimiter = 1.0;
    public static final double krotationLimiter = 1.0;

    /* Module Specific Constants */
    /* Front/intake Left Module  - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 19;
      public static final int angleMotorID = 18;
      public static final int canCoderID = 27;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(301.792);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front/intake Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 26;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(143.789);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back/arm Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 24;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(351.562);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back/arm Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 25;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(179.385);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        
    }
  }

}
