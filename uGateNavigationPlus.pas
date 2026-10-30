unit uGateNavigationPlus;

interface

uses uRecordedPlant;

const GatePlusHistoryCapacity = 1001;
type
  TGatePlusMode = (gpmGuidance, gpmTerminal, gpmPassage);

  TGatePlusParameters = record
    N, LeadTime, KLambda, MaxRate: Double;
    FilterTime, RateKp, RateKi, ActuatorTime: Double;
    SlipTime, SpeedKmh, FreezeDistance, InitialHeadingDegrees: Double;
    SensorDelay: Integer;
    SensorBias: Double;
    ObserverSlipTime, LeadFilterTime, MaxCommandAcceleration: Double;
    TerminalCommandAcceleration, TerminalMaxRate, PredictionTime: Double;
    SafeRate, DetectorHalfAngle, InitialSlipDegrees: Double;
  end;

  // Degrees, seconds; beta = body heading - velocity course.
  // Near/pass signals must come from an external source, never from angle alone.
  TGatePlusController = record
  private
    FHeadingHistory, FTimeHistory: array[0..GatePlusHistoryCapacity-1] of Double;
    FNext, FCount: Integer;
    FTime, FHeading, FPreviousDt, FSampleTime, FLOS, FIntegral: Double;
    FLeadRate, FLastSeen, FPlanRate, FPlanAcceleration: Double;
    FReady, FLost: Boolean;
  public
    LOSRate, LOSAcceleration, CorrectedAngle, CommandRate, Output: Double;
    EstimatedSlip, PredictedCourseChange, PredictionAge: Double;
    Mode: TGatePlusMode;
    TargetLost, PredictionExpired, PassageConfirmed: Boolean;
    procedure Reset;
    function Update(const P: TGatePlusParameters; Dt, Angle, Gyro: Double;
      MeasurementValid, TerminalRequested: Boolean;
      ConfirmPassage: Boolean = False): Double;
  end;

  // Independent plant and delayed detector; coordinates belong only to the simulator.
  TGatePlusSimulation = record
  private
    FAngles: array[0..GatePlusHistoryCapacity-1] of Double;
    FValid: array[0..GatePlusHistoryCapacity-1] of Boolean;
    FNext, FCount: Integer;
    FNearApproach, FPassed: Boolean;
  public
    RecordedPlant: TRecordedPlant;
    Parameters: TGatePlusParameters;
    Controller: TGatePlusController;
    X, Y, Heading, Course, Rate, MeasuredAngle, MinimumDistance: Double;
    procedure Reset(TargetX, TargetY: Double);
    procedure Step(Dt, TargetX, TargetY, TargetVX, TargetVY: Double;
      DetectorAvailable: Boolean = True; AngleNoiseDegrees: Double = 0);
  end;

function DefaultGatePlusParameters: TGatePlusParameters;

implementation

uses System.Math, System.SysUtils;

function WrapDegrees(A: Double): Double;
begin
  Result := RadToDeg(ArcTan2(Sin(DegToRad(A)), Cos(DegToRad(A))));
end;

function DefaultGatePlusParameters: TGatePlusParameters;
begin
  Result := Default(TGatePlusParameters);
  Result.N := 4;
  Result.LeadTime := 0.4;
  Result.KLambda := 0.5;
  Result.MaxRate := 90;
  Result.FilterTime := 0.15;
  Result.RateKp := 4;
  Result.RateKi := 20;
  Result.ActuatorTime := 0.05;
  Result.SlipTime := 0.4;
  Result.SpeedKmh := 150;
  Result.FreezeDistance := 5;
  Result.SensorDelay := 3;
  Result.ObserverSlipTime := 0.4;
  Result.LeadFilterTime := 0.15;
  Result.MaxCommandAcceleration := 360;
  Result.TerminalCommandAcceleration := 90;
  Result.TerminalMaxRate := 30;
  Result.PredictionTime := 0.3;
  Result.SafeRate := 15;
  Result.DetectorHalfAngle := 180;
end;

procedure TGatePlusController.Reset;
begin
  Self := Default(TGatePlusController);
end;

function TGatePlusController.Update(const P: TGatePlusParameters; Dt, Angle, Gyro: Double;
  MeasurementValid, TerminalRequested, ConfirmPassage: Boolean): Double;
var
  Index: Integer;
  SampleTime, SampleLOS, H, Alpha, Beta, Residual, Age, Decay: Double;
  Error, Candidate, Raw, Desired, Slew, Limit, PlanRate, Tau: Double;
  Accepted: Boolean;
begin
  if IsNan(Dt) or IsInfinite(Dt) or (Dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if (P.SensorDelay < 0) or (P.SensorDelay >= GatePlusHistoryCapacity) then
    raise EArgumentOutOfRangeException.Create('Gate sensor delay');
  if IsNan(Gyro) or IsInfinite(Gyro) then
    raise EArgumentException.Create('gyro must be finite');
  if Dt = 0 then Exit(Output);
  // The gyro describes the preceding plant interval, including variable dt.
  FHeading := FHeading + Gyro * FPreviousDt;
  Tau := Max(0.0, P.ObserverSlipTime);
  if Tau > 0 then
  begin
    Decay := Exp(-FPreviousDt / Tau);
    EstimatedSlip := Decay * EstimatedSlip + Tau * (1 - Decay) * Gyro;
  end
  else EstimatedSlip := 0;
  // Residual course rotation over the fixed forecast horizon if r becomes zero.
  if Tau > 0 then
    PredictedCourseChange := EstimatedSlip * (1 - Exp(-P.PredictionTime / Tau))
  else PredictedCourseChange := 0;

  FHeadingHistory[FNext] := FHeading;
  FTimeHistory[FNext] := FTime;
  Index := (FNext - P.SensorDelay + GatePlusHistoryCapacity) mod GatePlusHistoryCapacity;
  Accepted := MeasurementValid and (FCount >= P.SensorDelay) and
    not IsNan(Angle) and not IsInfinite(Angle);
  if TerminalRequested and (Mode = gpmGuidance) then Mode := gpmTerminal;
  PassageConfirmed := PassageConfirmed or ConfirmPassage;
  if PassageConfirmed then Mode := gpmPassage;
  if Accepted and not PassageConfirmed then
  begin
    SampleTime := FTimeHistory[Index];
    SampleLOS := Angle + FHeadingHistory[Index];
    if not FReady or (FLost and (FTime - FLastSeen > P.PredictionTime)) then
    begin
      // Reacquisition after an expired prediction must not differentiate a jump.
      FLOS := SampleLOS;
      LOSRate := 0;
      LOSAcceleration := 0;
      FLeadRate := 0;
      FSampleTime := SampleTime;
      FReady := True;
    end
    else
    begin
      H := SampleTime - FSampleTime;
      if H > 0 then
      begin
        Alpha := 1 - Exp(-H / Max(P.FilterTime, 0.000001));
        Beta := Sqr(Alpha) / (2 - Alpha);
        FLOS := FLOS + LOSRate * H;
        Residual := WrapDegrees(SampleLOS - FLOS);
        FLOS := FLOS + Alpha * Residual;
        LOSRate := LOSRate + Beta * Residual / H;
        // Bounded-bandwidth lead: s/(1 + Tf*s), no second difference of angle.
        H := Max(H, 0.000001);
        Alpha := 1 - Exp(-H / Max(P.LeadFilterTime, 0.000001));
        FLeadRate := FLeadRate + Alpha * (LOSRate - FLeadRate);
        LOSAcceleration := (LOSRate - FLeadRate) / Max(P.LeadFilterTime, 0.000001);
        FSampleTime := SampleTime;
      end;
    end;
    FLastSeen := FTime;
    FLost := False;
    if Mode = gpmPassage then Mode := gpmTerminal;
  end;
  TargetLost := not Accepted;
  PredictionAge := 0;
  if FReady then PredictionAge := Max(0.0, FTime - FLastSeen);
  PredictionExpired := not FReady or (PredictionAge > P.PredictionTime);
  if TargetLost and FReady and not FLost then
  begin
    // Capture a short course-rate plan. It is not a time-to-impact estimate.
    FPlanRate := EnsureRange(P.N * LOSRate, -P.TerminalMaxRate, P.TerminalMaxRate);
    FPlanAcceleration := EnsureRange(P.N * LOSAcceleration,
      -P.TerminalCommandAcceleration, P.TerminalCommandAcceleration);
    FLost := True;
  end;
  if TargetLost and FReady and (Mode = gpmTerminal) then Mode := gpmPassage;

  if FReady then
  begin
    Age := FLastSeen - FSampleTime + Min(PredictionAge, P.PredictionTime);
    CorrectedAngle := WrapDegrees(FLOS + LOSRate * Age - FHeading);
  end;
  Slew := P.MaxCommandAcceleration;
  Limit := P.MaxRate;
  if (Mode <> gpmGuidance) or TargetLost then
  begin
    Slew := Min(Slew, P.TerminalCommandAcceleration);
    Limit := Min(Limit, P.TerminalMaxRate);
  end;
  if PassageConfirmed or PredictionExpired then
  begin
    // Limited slip unloading after the plan expires. Zero yaw is NOT straight travel.
    Desired := EnsureRange(-EstimatedSlip / Max(Tau, 0.1), -P.SafeRate, P.SafeRate);
  end
  else if TargetLost then
  begin
    PlanRate := EnsureRange(FPlanRate + FPlanAcceleration * PredictionAge, -Limit, Limit);
    Desired := PlanRate + (Tau * PlanRate - EstimatedSlip) / Max(P.PredictionTime, 0.001);
  end
  else if Mode = gpmTerminal then
  begin
    // Track the desired course rate while accounting for stored lateral slip.
    PlanRate := EnsureRange(P.N * LOSRate, -Limit, Limit);
    Desired := PlanRate + (Tau * PlanRate - EstimatedSlip) / Max(P.PredictionTime, 0.001) +
      P.KLambda * CorrectedAngle;
  end
  else
    Desired := P.N * LOSRate + P.N * P.LeadTime * LOSAcceleration +
      P.KLambda * CorrectedAngle;
  Desired := EnsureRange(Desired, -Limit, Limit);
  CommandRate := EnsureRange(Desired, CommandRate - Slew * Dt, CommandRate + Slew * Dt);
  CommandRate := EnsureRange(CommandRate, -P.MaxRate, P.MaxRate);

  Error := CommandRate - Gyro;
  Candidate := EnsureRange(FIntegral + P.RateKi * Error * Dt, -P.MaxRate, P.MaxRate);
  Raw := P.RateKp * Error + Candidate;
  if (Abs(Raw) <= P.MaxRate) or (Raw * Error <= 0) then FIntegral := Candidate;
  Output := EnsureRange(P.RateKp * Error + FIntegral, -P.MaxRate, P.MaxRate);
  FNext := (FNext + 1) mod GatePlusHistoryCapacity;
  FCount := Min(FCount + 1, GatePlusHistoryCapacity);
  FTime := FTime + Dt;
  FPreviousDt := Dt;
  Result := Output;
end;

procedure TGatePlusSimulation.Reset(TargetX, TargetY: Double);
var P: TGatePlusParameters;
begin
  P := Parameters;
  Self := Default(TGatePlusSimulation);
  RecordedPlant.Parameters := DefaultRecordedPlantParameters;
  RecordedPlant.Reset;
  Parameters := P;
  Heading := WrapDegrees(P.InitialHeadingDegrees);
  Course := WrapDegrees(Heading - P.InitialSlipDegrees);
  if RecordedPlant.Enabled then Course := Heading;
  MinimumDistance := Hypot(TargetX, TargetY);
end;

procedure TGatePlusSimulation.Step(Dt, TargetX, TargetY, TargetVX, TargetVY: Double;
  DetectorAvailable: Boolean; AngleNoiseDegrees: Double);
var
  Angle, Control, Gain, DX, DY, RX, RY, Fraction, Denom, ClosingDot: Double;
  Index: Integer;
  Valid, NearClosing: Boolean;
  Gyro: Double;
begin
  if IsNan(Dt) or IsInfinite(Dt) or (Dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if (Parameters.SensorDelay < 0) or (Parameters.SensorDelay >= GatePlusHistoryCapacity) then
    raise EArgumentOutOfRangeException.Create('Gate sensor delay');
  if Dt = 0 then Exit;
  DX := TargetX - X; DY := TargetY - Y;
  Angle := WrapDegrees(RadToDeg(ArcTan2(DY, DX)) - Heading);
  FValid[FNext] := DetectorAvailable and (Abs(Angle) <= Parameters.DetectorHalfAngle);
  FAngles[FNext] := WrapDegrees(Angle + Parameters.SensorBias + AngleNoiseDegrees);
  Index := (FNext - Parameters.SensorDelay + GatePlusHistoryCapacity) mod GatePlusHistoryCapacity;
  Valid := (FCount >= Parameters.SensorDelay) and FValid[Index];
  MeasuredAngle := FAngles[Index];
  Gyro := Rate;
  if RecordedPlant.Enabled then
  begin
    MeasuredAngle := RecordedPlant.Observe(Angle + AngleNoiseDegrees, FValid[FNext]);
    Valid := RecordedPlant.MeasurementValid;
    Gyro := RecordedPlant.GyroRate;
  end;
  ClosingDot := DX * (TargetVX - Parameters.SpeedKmh / 3.6 * Cos(DegToRad(Course))) +
    DY * (TargetVY - Parameters.SpeedKmh / 3.6 * Sin(DegToRad(Course)));
  // Simulator-only evidence: near AND approaching, then a closest-approach event.
  NearClosing := (Parameters.FreezeDistance > 0) and (ClosingDot < 0) and
    (Hypot(DX, DY) <= Parameters.FreezeDistance);
  FNearApproach := FNearApproach or NearClosing;
  FPassed := FPassed or (FNearApproach and (ClosingDot >= 0));
  Control := Controller.Update(Parameters, Dt, MeasuredAngle, Gyro, Valid,
    NearClosing, FPassed);
  FNext := (FNext + 1) mod GatePlusHistoryCapacity;
  FCount := Min(FCount + 1, GatePlusHistoryCapacity);
  Gain := 1 - Exp(-Dt / Max(Parameters.ActuatorTime, 0.000001));
  if RecordedPlant.Enabled then Rate := RecordedPlant.Advance(Control, Dt)
  else Rate := Rate + Gain * (Control - Rate);
  Heading := WrapDegrees(Heading + Rate * Dt);
  if RecordedPlant.Enabled then Course := RecordedPlant.AdvanceCourse(Heading, Course, Dt)
  else if Parameters.SlipTime = 0 then Course := Heading
  else Course := WrapDegrees(Course + (1 - Exp(-Dt / Parameters.SlipTime)) *
    WrapDegrees(Heading - Course));
  // Constant speed approximation; no throttle reduction in turns.
  RX := (TargetVX - Parameters.SpeedKmh / 3.6 * Cos(DegToRad(Course))) * Dt;
  RY := (TargetVY - Parameters.SpeedKmh / 3.6 * Sin(DegToRad(Course))) * Dt;
  Denom := Sqr(RX) + Sqr(RY);
  Fraction := 0;
  if Denom > 0 then Fraction := EnsureRange(-(DX * RX + DY * RY) / Denom, 0.0, 1.0);
  MinimumDistance := Min(MinimumDistance, Hypot(DX + Fraction * RX, DY + Fraction * RY));
  X := X + Parameters.SpeedKmh / 3.6 * Cos(DegToRad(Course)) * Dt;
  Y := Y + Parameters.SpeedKmh / 3.6 * Sin(DegToRad(Course)) * Dt;
end;

end.