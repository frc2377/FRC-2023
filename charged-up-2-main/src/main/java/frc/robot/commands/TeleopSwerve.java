package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//command for teleop swerve

public class TeleopSwerve extends CommandBase {
  private SwerveSubsystem s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Constants.Swerve.ktranslationLimiter);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.Swerve.kstrafeLimiter);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.Swerve.krotationLimiter);
 
  public TeleopSwerve(
      SwerveSubsystem s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup
      ) 
      
  {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;  
  }

  @Override
  public void execute() {

    SmartDashboard.putNumber("Pitch", s_Swerve.getPitch());
    SmartDashboard.putNumber("Yaw", s_Swerve.getYaw().getDegrees());
    SmartDashboard.putNumber("Roll", s_Swerve.getRoll());
    SmartDashboard.putNumber("X", s_Swerve.getPose().getX());
    SmartDashboard.putNumber("Y", s_Swerve.getPose().getY());    
    
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  } 
}
