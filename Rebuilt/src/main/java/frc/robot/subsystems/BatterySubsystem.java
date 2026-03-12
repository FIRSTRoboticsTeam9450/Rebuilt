// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BattFuelGaugeFixed;
import frc.robot.util.LoggingUtils;

public class BatterySubsystem extends SubsystemBase {
  private static BatterySubsystem INSTANCE;

  private final BattFuelGaugeFixed BFG = new BattFuelGaugeFixed(80);

  private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("Tuning/Debugging Toggles/Battery", false);
  private final LoggedNetworkBoolean logDebuggingToggleSecondary = new LoggedNetworkBoolean("Tuning/Debugging Toggles/Battery Secondary", false); // For less important values (like firmware versions)

  /** Creates a new BatterySubsystem. */
  public BatterySubsystem() {
  }
  public static BatterySubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new BatterySubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    // No indents = Not shown on BFG screen. One indent = Partially shown on BFG screen. Two indents = Shown on BFG screen. (From base indent level)
    if (logDebuggingToggle.get()) {
          Logger.recordOutput("TankDrive/BatterySubsystem/Nickname_Str", BFG.getNickname());
          Logger.recordOutput("TankDrive/BatterySubsystem/Manufacturer_Enum", BFG.getManufacturer());
          float capacityAh = BFG.getCapacityAh();
          Logger.recordOutput("TankDrive/BatterySubsystem/Capacity_Ah", capacityAh);
          Logger.recordOutput("TankDrive/BatterySubsystem/RatedCapacity_Ah", BFG.getRatedCapacityAh());
          Logger.recordOutput("TankDrive/BatterySubsystem/Age_Days", BFG.getBatteryAgeDays()); // Not actually in days, don't know why. Returns 4.3e16
          Logger.recordOutput("TankDrive/BatterySubsystem/NumCycles_Num", BFG.getNumCycles());

          Logger.recordOutput("TankDrive/BatterySubsystem/Voltage_V", BFG.getVoltage());
          Logger.recordOutput("TankDrive/BatterySubsystem/Current_A", BFG.getCurrent());
        Logger.recordOutput("TankDrive/BatterySubsystem/State_Enum", BFG.getBatteryState());
          float effectiveCapacityAh = BFG.getEffectiveCapacityAh();
          Logger.recordOutput("TankDrive/BatterySubsystem/EffectiveCapacity_Ah", effectiveCapacityAh);
          float effectiveCapacityPct = effectiveCapacityAh / capacityAh * 100.0F;
          Logger.recordOutput("TankDrive/BatterySubsystem/EffectiveCapacity_Pct", effectiveCapacityPct);
      float depthOfDischargeAh = BFG.getDodAh();
      Logger.recordOutput("TankDrive/BatterySubsystem/DepthOfDischarge_Ah", depthOfDischargeAh);
      Logger.recordOutput("TankDrive/BatterySubsystem/DepthOfDischarge_Wh", BFG.getDodWh());
        float stateOfChargeAh = effectiveCapacityAh - depthOfDischargeAh; // Labeled as "remaining charge" in docs
        Logger.recordOutput("TankDrive/BatterySubsystem/StateOfCharge_Ah", stateOfChargeAh);
        float stateOfChargePct = BFG.getRemainingChargePct();
        Logger.recordOutput("TankDrive/BatterySubsystem/StateOfCharge_Pct", stateOfChargePct);


          Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinVoltage_V", BFG.getCycleMinVoltage());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxVoltage_V", BFG.getCycleMaxVoltage());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinCurrent_A", BFG.getCycleMinCurrent());
          Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxCurrent_A", BFG.getCycleMaxCurrent());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/StartingVoltage_V", BFG.getCycleStartingVoltage());
          Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/RmsDischargeCurrent_A", BFG.getCycleRmsDischargeCurrent());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinDepthOfDischarge_Ah", BFG.getCycleMinDodAh());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinDepthOfDischarge_Wh", BFG.getCycleMinDodWh()); // Returns the same value as the current cycle min depth of discharge in amp-hours, but shouldn't
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxDepthOfDischarge_Ah", BFG.getCycleMaxDodAh());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxDepthOfDischarge_Wh", BFG.getCycleMaxDodWh());
        LoggingUtils.logSecondsAsTime("TankDrive/BatterySubsystem/CurrentCycle/ChargeTime_Time", (int) BFG.getCycleChargeTime());
        LoggingUtils.logSecondsAsTime("TankDrive/BatterySubsystem/CurrentCycle/DischargeTime_Time", (int) BFG.getCycleDischargeTime());


          Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MinVoltage_V", BFG.getMatchMinVoltage());
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MaxVoltage_V", BFG.getMatchMaxVoltage());
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MinCurrent_A", BFG.getMatchMinCurrent());
          Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MaxCurrent_A", BFG.getMatchMaxCurrent());
          Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/StartingVoltage_V", BFG.getMatchStartingVoltage());
          Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/RmsDischargeCurrent_A", BFG.getMatchRmsDischargeCurrent());
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MinDepthOfDischarge_Ah", BFG.getMatchMinDodAh());
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MaxDepthOfDischarge_Ah", BFG.getMatchMaxDodAh());
      LoggingUtils.logSecondsAsTime("TankDrive/BatterySubsystem/LastMatch/ChargeTime_Time", (int) BFG.getMatchChargeTime());
      LoggingUtils.logSecondsAsTime("TankDrive/BatterySubsystem/LastMatch/DischargeTime_Time", (int) BFG.getMatchDischargeTime());
          LoggingUtils.logSecondsAsTime("TankDrive/BatterySubsystem/LastMatch/Duration_Time", (int) BFG.getMatchDuration()); // Labeled as "active time" on webpage
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/StartingCharge_Ah", BFG.getMatchStartingCharge()); // Webpage calls this "Starting depth of discharge"
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/StartingCharge_Wh", BFG.getMatchStartingEnergy()); // Webpage calls this "Starting depth of discharge"
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/EndingCharge_Ah", BFG.getMatchEndingCharge()); // Webpage calls this "Ending depth of discharge"
      Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/EndingCharge_Wh", BFG.getMatchEndingEnergy()); // Webpage calls this "Ending depth of discharge"
    }
    if (logDebuggingToggleSecondary.get()) {
      Logger.recordOutput("TankDrive/BatterySubsystem/SerialNumber_Num", BFG.getSerialNumber());
      Logger.recordOutput("TankDrive/BatterySubsystem/FirmwareVersion_Num", BFG.getFirmwareVersion());
    }


    // Shown on BFG but cannot get in code (or some of these would instead require math that I do not currently know about):
      // Match:
        // Discharging charge (Ah)
        // Discharging charge (kJ)
      // Cycle / LastCycle:
        // Charging charge (Ah)
        // Charging charge (kJ)
        // Discharging charge (Ah)
        // Discharging charge (kJ)
      // Lifetime:
        // Min voltage (V)
        // Max discharging charge (Ah)
        // Max discharging charge (kJ)
  }
}
