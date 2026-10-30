unit uRecordedPlant;

interface

const RecordedHistorySize = 512;

type
  TRecordedPlantParameters = record
    Profile: Integer; // 0: legacy, 1: EDF Roll, 2: EDF Pitch, 3: Hovercraft Roll.
    GainScale, ExtraDelay, FrequencyScale, DampingScale: Double;
    OutputLimit: Double; // Controller units; 120 preserves the existing profiles.
    CameraHz, CameraDelay, CompensationTime, NoiseScale, CameraBias: Double;
    Seed: Cardinal;
    CourseTime: Double; // Geometric assumption, NOT identified from the EDF.
  end;

  // Common causal plant/sensor pipeline for all four controllers. Degrees/seconds.
  TRecordedPlant = record
  private
    FTimes, FCommands, FAngles: array[0..RecordedHistorySize-1] of Double;
    FValid: array[0..RecordedHistorySize-1] of Boolean;
    FNext, FCount: Integer;
    FTime, FAcceleration, FNextCamera, FLastObservation, FHeading, FSampleHeading: Double;
    FRandom: Cardinal;
    function Historical(Time: Double; Camera: Boolean; out Valid: Boolean): Double;
    function Noise: Double;
  public
    Parameters: TRecordedPlantParameters;
    Rate, GyroRate, RawAngle, CameraAngle, MeasuredAngle, LastCommand: Double;
    MeasurementValid, FreshMeasurement: Boolean;
    procedure Reset(InitialRate: Double = 0);
    function Enabled: Boolean;
    procedure Coefficients(out Linear, Cubic, Frequency, Damping, Delay,
      CameraSigma: Double);
    function SteadyRate(Command: Double): Double;
    function Observe(TrueAngle: Double; Available: Boolean = True): Double;
    function Advance(Command, Dt: Double): Double;
    function AdvanceCourse(Heading, Course, Dt: Double): Double;
  end;

function DefaultRecordedPlantParameters: TRecordedPlantParameters;

implementation

uses System.Math, System.SysUtils;

function Wrap(X: Double): Double;
begin
  Result := RadToDeg(ArcTan2(Sin(DegToRad(X)), Cos(DegToRad(X))));
end;

function DefaultRecordedPlantParameters: TRecordedPlantParameters;
begin
  Result := Default(TRecordedPlantParameters);
  Result.GainScale := 1;
  Result.OutputLimit := 120;
  Result.FrequencyScale := 1;
  Result.DampingScale := 1;
  Result.CameraHz := 60;
  Result.CameraDelay := 0.035;
  Result.CompensationTime := 0.035;
  Result.NoiseScale := 1;
  Result.Seed := 20261023;
  Result.CourseTime := 0.4;
end;

function TRecordedPlant.Enabled: Boolean;
begin
  Result := Parameters.Profile <> 0;
end;

procedure TRecordedPlant.Reset(InitialRate: Double);
var P: TRecordedPlantParameters;
begin
  P := Parameters;
  Self := Default(TRecordedPlant);
  Parameters := P;
  Rate := InitialRate;
  GyroRate := InitialRate;
  FLastObservation := -1;
  FRandom := P.Seed;
end;

procedure TRecordedPlant.Coefficients(out Linear, Cubic, Frequency, Damping,
  Delay, CameraSigma: Double);
var V: Double;
begin
  for V in TArray<Double>.Create(Parameters.GainScale, Parameters.ExtraDelay, Parameters.FrequencyScale,
    Parameters.DampingScale, Parameters.CameraHz, Parameters.CameraDelay,
    Parameters.CompensationTime, Parameters.NoiseScale, Parameters.CourseTime, Parameters.CameraBias, Parameters.OutputLimit) do
    if IsNan(V) or IsInfinite(V) then
      raise EArgumentException.Create('Non-finite recorded parameter');
  // Static curve: robust manual RC bins. Dynamics: first 1.12 s of Gate only.
  // The remaining 0.94 s is held out, with substantial residual model error.
  case Parameters.Profile of
    1: begin
      Linear := 1.27750595; Cubic := 0.00112717235;
      Frequency := 6.75; Damping := 0.5; Delay := 0.07;
      CameraSigma := 0.18;
    end;
    2: begin
      Linear := 1.41521828; Cubic := 0.00130267125;
      Frequency := 4.5; Damping := 0.575; Delay := 0.06;
      CameraSigma := 0.13;
    end;
    3: begin
      // Hovercraft LOG0005..0064, whole-record odd/even fit/validation split.
      // Command units map to RC by 1500+2.5*Command. Static gain per command=0.4852524.
      Linear := 0.9705048326214148; Cubic := 0;
      Frequency := 1.4843806631280174; Damping := 0.31505869989860313;
      Delay := 0.06; CameraSigma := 0.10; // Camera noise is a scenario assumption.
    end;
    else raise EArgumentOutOfRangeException.Create('Recorded profile must be 1, 2 or 3');
  end;
  Frequency := Frequency * Parameters.FrequencyScale;
  Damping := Damping * Parameters.DampingScale;
  Delay := Delay + Parameters.ExtraDelay;
  CameraSigma := CameraSigma * Parameters.NoiseScale;
  if (Parameters.OutputLimit <= 0) or (Parameters.OutputLimit > 120) or (Frequency <= 0) or (Damping <= 0) or (Damping >= 1) or
    (Delay < 0) or (Delay > 0.5) or (Parameters.GainScale <= 0) or
    (Parameters.CameraHz <= 0) or (Parameters.CameraDelay < 0) or
    (Parameters.CameraDelay > 0.5) or (Parameters.CourseTime < 0) or
    (Parameters.NoiseScale < 0) then
    raise EArgumentOutOfRangeException.Create('Invalid recorded-plant parameters');
end;

function TRecordedPlant.SteadyRate(Command: Double): Double;
var L, C, F, Z, D, S, Percent: Double;
begin
  Coefficients(L, C, F, Z, D, S);
  // Existing firmware clamps to +/-120 then maps 2.5 us per nominal deg/s.
  // Thus percent of a 500-us half-range is command/2, NOT actual angular rate.
  Percent := EnsureRange(Command, -120.0, 120.0) / 2;
  Result := Parameters.GainScale * (L * Percent + C * Percent * Percent * Percent);
end;

function TRecordedPlant.Historical(Time: Double; Camera: Boolean;
  out Valid: Boolean): Double;
var I, J, K: Integer; A: Double;
begin
  Result := 0; Valid := False;
  // Current observation/command is stored before calling this routine.
  for K := 0 to FCount do
  begin
    I := (FNext - K + RecordedHistorySize) mod RecordedHistorySize;
    if FTimes[I] <= Time + 1E-10 then
    begin
      if not Camera then begin Valid := True; Exit(FCommands[I]); end;
      Valid := FValid[I]; Result := FAngles[I];
      // Interpolate wrapped angle in physical time, never command into the future.
      if K > 0 then
      begin
        J := (I + 1) mod RecordedHistorySize;
        if FTimes[J] > FTimes[I] then
        begin
          A := EnsureRange((Time-FTimes[I])/(FTimes[J]-FTimes[I]), 0.0, 1.0);
          Result := Wrap(FAngles[I] + A * Wrap(FAngles[J]-FAngles[I]));
          Valid := Valid and FValid[J];
        end;
      end;
      Exit;
    end;
  end;
  if FCount = RecordedHistorySize-1 then
    raise EInvalidOp.Create('Recorded history too short for selected delay/dt');
  // At reset assume a constant prehistory angle, and zero preceding command.
  if Camera then begin Result := FAngles[0]; Valid := FValid[0]; end;
end;

function TRecordedPlant.Noise: Double;
var U1, U2: Double;
begin
  // Local generator: reproducible after reset and independent of other models.
  FRandom := Cardinal((UInt64(FRandom)*1664525+1013904223) and $FFFFFFFF);
  U1 := (FRandom+0.5)/4294967296.0;
  FRandom := Cardinal((UInt64(FRandom)*1664525+1013904223) and $FFFFFFFF);
  U2 := (FRandom+0.5)/4294967296.0;
  Result := Sqrt(-2*Ln(U1))*Cos(2*Pi*U2);
end;

function TRecordedPlant.Observe(TrueAngle: Double; Available: Boolean): Double;
var L, C, F, Z, D, Sigma: Double; Valid: Boolean;
begin
  Coefficients(L, C, F, Z, D, Sigma);
  if IsNan(TrueAngle) or IsInfinite(TrueAngle) then
    raise EArgumentException.Create('Non-finite sensor angle');
  if FLastObservation = FTime then Exit(MeasuredAngle);
  FLastObservation := FTime;
  FTimes[FNext] := FTime; FAngles[FNext] := Wrap(TrueAngle);
  FValid[FNext] := Available;
  GyroRate := Rate; // Effective high-frequency disturbance, not isolated sensor noise.
  if Parameters.Profile = 3 then GyroRate := Rate + 5.0*Parameters.NoiseScale*Noise;
  FreshMeasurement := FTime + 1E-10 >= FNextCamera;
  if FreshMeasurement then
  begin
    RawAngle := Historical(FTime-Parameters.CameraDelay, True, Valid);
    RawAngle := Round((RawAngle + Parameters.CameraBias + Sigma*Noise)*100)/100;
    MeasurementValid := Valid;
    CameraAngle := Wrap(RawAngle-GyroRate*Parameters.CompensationTime);
    FSampleHeading := FHeading;
    repeat FNextCamera := FNextCamera + 1/Parameters.CameraHz
    until FNextCamera > FTime + 1E-10;
  end;
  MeasuredAngle := Wrap(CameraAngle-(FHeading-FSampleHeading));
  Result := MeasuredAngle;
end;

function TRecordedPlant.Advance(Command, Dt: Double): Double;
var L, C, F, Z, Delay, Sigma, Target, W, WD, Decay, CS, SN,
  Error, NewRate: Double; Valid: Boolean;
begin
  if IsNan(Dt) or IsInfinite(Dt) or (Dt < 0) or (Dt > 0.02) then
    raise EArgumentException.Create('Recorded plant requires 0 <= dt <= 0.02 s');
  if IsNan(Command) or IsInfinite(Command) then
    raise EArgumentException.Create('Non-finite command');
  if Dt = 0 then Exit(Rate);
  Coefficients(L, C, F, Z, Delay, Sigma);
  FTimes[FNext] := FTime;
  LastCommand := EnsureRange(Command, -Parameters.OutputLimit, Parameters.OutputLimit);
  FCommands[FNext] := LastCommand;
  Target := SteadyRate(Historical(FTime-Delay, False, Valid));
  W := 2*Pi*F; WD := W*Sqrt(1-Z*Z);
  Decay := Exp(-Z*W*Dt); CS := Cos(WD*Dt); SN := Sin(WD*Dt);
  Error := Rate-Target;
  NewRate := Target + Decay*(Error*CS+(FAcceleration+Z*W*Error)*SN/WD);
  FAcceleration := Decay*(FAcceleration*CS-(Z*W*FAcceleration+W*W*Error)*SN/WD);
  Rate := NewRate; GyroRate := Rate;
  FHeading := FHeading + Rate*Dt;
  FTime := FTime + Dt;
  FNext := (FNext+1) mod RecordedHistorySize;
  FCount := Min(FCount+1, RecordedHistorySize-1);
  Result := Rate;
end;

function TRecordedPlant.AdvanceCourse(Heading, Course, Dt: Double): Double;
begin
  if Parameters.CourseTime = 0 then Exit(Heading);
  Result := Wrap(Course+(1-Exp(-Dt/Parameters.CourseTime))*Wrap(Heading-Course));
end;

end.
