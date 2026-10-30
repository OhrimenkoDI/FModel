unit uGateNavigation;

interface

uses uRecordedPlant;

const GateHistoryCapacity = 1001;
type
  TGateParameters = record
    N, LeadTime, KLambda, MaxRate: Double;
    FilterTime, RateKp, RateKi, ActuatorTime: Double;
    SlipTime, SpeedKmh, FreezeDistance, InitialHeadingDegrees: Double;
    SensorDelay: Integer;
    SensorBias: Double;
  end;

  // Degrees, seconds. No position, range or speed input to the guidance law.
  TGateController = record
  private
    FHeadingHistory, FTimeHistory: array[0..GateHistoryCapacity-1] of Double;
    FNext, FCount: Integer;
    FTime, FHeading, FPreviousDt, FSampleTime, FLOS, FIntegral: Double;
    FReady: Boolean;
  public
    LOSRate, LOSAcceleration, CorrectedAngle, CommandRate, Output: Double;
    Frozen: Boolean;
    procedure Reset;
    function Update(const P: TGateParameters; Dt, Angle, Gyro: Double;
      MeasurementValid, FreezeRequested: Boolean): Double;
  end;

  // Independent plant and delayed detector; target coordinates belong to the simulator.
  TGateSimulation = record
  private
    FAngles: array[0..GateHistoryCapacity-1] of Double;
    FNext, FCount: Integer;
  public
    RecordedPlant: TRecordedPlant;
    Parameters: TGateParameters;
    Controller: TGateController;
    X, Y, Heading, Course, Rate, MeasuredAngle, MinimumDistance: Double;
    procedure Reset(TargetX, TargetY: Double);
    procedure Step(Dt, TargetX, TargetY, TargetVX, TargetVY: Double);
  end;

function DefaultGateParameters: TGateParameters;

implementation

uses System.Math, System.SysUtils;

function WrapDegrees(A: Double): Double;
begin
  Result := RadToDeg(ArcTan2(Sin(DegToRad(A)), Cos(DegToRad(A))));
end;

function DefaultGateParameters: TGateParameters;
begin
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
  Result.InitialHeadingDegrees := 0;
  Result.FreezeDistance := 5;
  Result.SensorDelay := 3;
  Result.SensorBias := 0;
end;

procedure TGateController.Reset;
begin
  Self := Default(TGateController);
end;

function TGateController.Update(const P: TGateParameters; Dt, Angle, Gyro: Double;
  MeasurementValid, FreezeRequested: Boolean): Double;
var
  Index: Integer;
  SampleTime, SampleLOS, H, Alpha, Beta, Residual, OldRate, Age: Double;
  Error, Candidate, Raw: Double;
begin
  if IsNan(Dt) or IsInfinite(Dt) or (Dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if (P.SensorDelay < 0) or (P.SensorDelay >= GateHistoryCapacity) then
    raise EArgumentOutOfRangeException.Create('Gate sensor delay');
  if Dt = 0 then Exit(Output);
  // Gyro is the rate used by the plant during the preceding interval.
  FHeading := FHeading + Gyro * FPreviousDt;
  FHeadingHistory[FNext] := FHeading;
  FTimeHistory[FNext] := FTime;
  Index := (FNext - P.SensorDelay + GateHistoryCapacity) mod GateHistoryCapacity;
  if MeasurementValid and (FCount >= P.SensorDelay) then
  begin
    SampleTime := FTimeHistory[Index];
    SampleLOS := Angle + FHeadingHistory[Index];
    if not FReady then
    begin
      FLOS := SampleLOS;
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
        OldRate := LOSRate;
        LOSRate := LOSRate + Beta * Residual / H;
        LOSAcceleration := LOSAcceleration + Alpha *
          ((LOSRate - OldRate) / H - LOSAcceleration);
        FSampleTime := SampleTime;
      end;
    end;
    // Restore inertial bearing with gyro history, then predict through detector delay.
    Age := FTime - SampleTime;
    CorrectedAngle := WrapDegrees(FLOS + LOSRate * Age - FHeading);
    if not Frozen and not FreezeRequested then
      CommandRate := EnsureRange(P.N * LOSRate + P.N * P.LeadTime * LOSAcceleration +
        P.KLambda * CorrectedAngle, -P.MaxRate, P.MaxRate);
  end;
  Frozen := Frozen or FreezeRequested;
  Error := CommandRate - Gyro;
  Candidate := EnsureRange(FIntegral + P.RateKi * Error * Dt, -P.MaxRate, P.MaxRate);
  Raw := P.RateKp * Error + Candidate;
  // Conditional integration prevents wind-up at the steering actuator limit.
  if (Abs(Raw) <= P.MaxRate) or (Raw * Error <= 0) then FIntegral := Candidate;
  Output := EnsureRange(P.RateKp * Error + FIntegral, -P.MaxRate, P.MaxRate);
  FNext := (FNext + 1) mod GateHistoryCapacity;
  FCount := Min(FCount + 1, GateHistoryCapacity);
  FTime := FTime + Dt;
  FPreviousDt := Dt;
  Result := Output;
end;

procedure TGateSimulation.Reset(TargetX, TargetY: Double);
var P: TGateParameters;
begin
  P := Parameters;
  Self := Default(TGateSimulation);
  RecordedPlant.Parameters := DefaultRecordedPlantParameters;
  RecordedPlant.Reset;
  Parameters := P;
  Heading := WrapDegrees(P.InitialHeadingDegrees);
  Course := Heading;
  MinimumDistance := Hypot(TargetX, TargetY);
end;

procedure TGateSimulation.Step(Dt, TargetX, TargetY, TargetVX, TargetVY: Double);
var
  Angle, Control, Gain, DX, DY, RX, RY, Fraction, Denom: Double;
  Index: Integer;
  Valid: Boolean;
  Gyro: Double;
begin
  if IsNan(Dt) or IsInfinite(Dt) or (Dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if (Parameters.SensorDelay < 0) or (Parameters.SensorDelay >= GateHistoryCapacity) then
    raise EArgumentOutOfRangeException.Create('Gate sensor delay');
  if Dt = 0 then Exit;
  DX := TargetX - X; DY := TargetY - Y;
  Angle := WrapDegrees(RadToDeg(ArcTan2(DY, DX)) - Heading + Parameters.SensorBias);
  FAngles[FNext] := Angle;
  Index := (FNext - Parameters.SensorDelay + GateHistoryCapacity) mod GateHistoryCapacity;
  Valid := FCount >= Parameters.SensorDelay;
  MeasuredAngle := FAngles[Index];
  Gyro := Rate;
  if RecordedPlant.Enabled then
  begin
    MeasuredAngle := RecordedPlant.Observe(Angle-Parameters.SensorBias);
    Valid := RecordedPlant.MeasurementValid;
    Gyro := RecordedPlant.GyroRate;
  end;
  Control := Controller.Update(Parameters, Dt, MeasuredAngle, Gyro, Valid,
    (Parameters.FreezeDistance > 0) and (Hypot(DX, DY) <= Parameters.FreezeDistance));
  FNext := (FNext + 1) mod GateHistoryCapacity;
  FCount := Min(FCount + 1, GateHistoryCapacity);
  Gain := 1 - Exp(-Dt / Max(Parameters.ActuatorTime, 0.000001));
  if RecordedPlant.Enabled then Rate := RecordedPlant.Advance(Control, Dt)
  else Rate := Rate + Gain * (Control - Rate);
  Heading := WrapDegrees(Heading + Rate * Dt);
  if RecordedPlant.Enabled then Course := RecordedPlant.AdvanceCourse(Heading, Course, Dt)
  else if Parameters.SlipTime = 0 then Course := Heading
  else Course := WrapDegrees(Course + (1 - Exp(-Dt / Parameters.SlipTime)) *
    WrapDegrees(Heading - Course));
  // Constant forward thrust/speed approximation: no throttle reduction in turns.
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
