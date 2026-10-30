unit uAdaptiveGlide;

// Port of adaptive-glide/Cxx_implementation/src/adaptive_controller.cpp.
// Planar adapter: angle + gyro only; no aircraft dynamics or surface actuators.
interface

type
  TAdaptiveVector = array[0..3] of Double;
  TAdaptiveMatrix = array[0..3,0..3] of Double;
  TAdaptiveParameters = record
    CommandLimit, SlidingGain, SwitchingGain, Kappa, Lambda: Double;
    IdentifierAlpha, CovarianceScale, DenominatorGuard: Double;
    RateGain, AngleLimit: Double; // r(rad/s) = RateGain * normalized original command(rad).
    ReferenceCoefficients, IdentifierFilter: array[0..2] of Double;
  end;
  TAdaptiveState = record
  private
    FCovariance: TAdaptiveMatrix;
    FFilterY, FFilterU: array[0..2] of Double;
    FPrefilter: TAdaptiveVector;
    FShunt, FPreviousDt: Double;
    function Calculate(Dt, AngleRadians: Double): Double;
  public
    Parameters: TAdaptiveParameters;
    Theta: TAdaptiveVector;
    Heading, RawReference, FilteredReference, SlidingSurface, Residual: Double; // radians
    Command, Output: Double; // original command, rate output in degrees/second
    procedure Init;
    procedure Reset;
    function Update(Dt, MeasuredAngleDegrees, GyroDegreesPerSecond: Double): Double;
  end;
function DefaultAdaptiveParameters: TAdaptiveParameters;

implementation
uses System.Math, System.SysUtils;

function DefaultAdaptiveParameters: TAdaptiveParameters;
begin
  Result.CommandLimit := 0.30;
  Result.SlidingGain := 9.62;
  Result.SwitchingGain := 0.00475;
  Result.Kappa := -0.136;
  Result.Lambda := 1.56;
  Result.IdentifierAlpha := 0.001;
  Result.CovarianceScale := 0.001;
  Result.DenominatorGuard := 0.35;
  Result.RateGain := -5; // Negative plant gain matches the source sign convention.
  Result.AngleLimit := 0.68;
  Result.ReferenceCoefficients[0] := 47.74;
  Result.ReferenceCoefficients[1] := 576.54;
  Result.ReferenceCoefficients[2] := 3420.84;
  Result.IdentifierFilter[0] := 20;
  Result.IdentifierFilter[1] := 200;
  Result.IdentifierFilter[2] := 1000;
end;

function Solve4(const Input: TAdaptiveMatrix; const RHS: TAdaptiveVector;
  out Solution: TAdaptiveVector): Boolean;
var A: array[0..3,0..4] of Double; R,C,J,Pivot: Integer; Temp, Scale, Factor: Double;
begin
  for R:=0 to 3 do
  begin
    for C:=0 to 3 do A[R,C]:=Input[R,C];
    A[R,4]:=RHS[R];
  end;
  for C:=0 to 3 do
  begin
    Pivot:=C;
    for R:=C+1 to 3 do if Abs(A[R,C])>Abs(A[Pivot,C]) then Pivot:=R;
    if Abs(A[Pivot,C])<1E-18 then Exit(False);
    if Pivot<>C then for J:=C to 4 do
    begin Temp:=A[C,J]; A[C,J]:=A[Pivot,J]; A[Pivot,J]:=Temp; end;
    Scale:=A[C,C];
    for J:=C to 4 do A[C,J]:=A[C,J]/Scale;
    for R:=0 to 3 do if R<>C then
    begin
      Factor:=A[R,C];
      for J:=C to 4 do A[R,J]:=A[R,J]-Factor*A[C,J];
    end;
  end;
  for R:=0 to 3 do Solution[R]:=A[R,4];
  Result:=True;
end;

procedure TAdaptiveState.Init;
begin
  Parameters:=DefaultAdaptiveParameters;
  Reset;
end;

procedure TAdaptiveState.Reset;
var P:TAdaptiveParameters; I:Integer;
begin
  P:=Parameters;
  Self:=Default(TAdaptiveState);
  Parameters:=P;
  Theta[3]:=-10;
  if P.CovarianceScale<=0 then raise EArgumentException.Create('Adaptive covariance scale must be positive');
  for I:=0 to 3 do FCovariance[I,I]:=1/P.CovarianceScale;
end;

function TAdaptiveState.Calculate(Dt, AngleRadians: Double): Double;
const Limits:TAdaptiveVector=(80,500,200,500);
var Am1,Am2,Am3,B1,Gain,Y3,U3,Updated,Decay,Input:Double;
  Dx,Phi,Solved:TAdaptiveVector;
  Dy,Du:array[0..2] of Double;
  I,J:Integer;
begin
  RawReference:=Heading+EnsureRange(ArcTan2(Sin(AngleRadians),Cos(AngleRadians)),
    -Parameters.AngleLimit,Parameters.AngleLimit);
  Am1:=Parameters.ReferenceCoefficients[0];
  Am2:=Parameters.ReferenceCoefficients[1];
  Am3:=Parameters.ReferenceCoefficients[2];
  B1:=Theta[3];
  if Abs(B1)<Parameters.DenominatorGuard then
    if B1<0 then B1:=-Parameters.DenominatorGuard else B1:=Parameters.DenominatorGuard;
  Gain:=Am3/B1;
  FilteredReference:=Gain*(B1*Parameters.Lambda*FPrefilter[0]+
    (Parameters.Kappa*Theta[1]+Theta[2]*Parameters.Lambda+B1)*FPrefilter[1]+
    (Parameters.Kappa*Theta[0]+Theta[2])*FPrefilter[2]+Parameters.Kappa*FPrefilter[3]);
  Dx[0]:=FPrefilter[1]; Dx[1]:=FPrefilter[2]; Dx[2]:=FPrefilter[3];
  Dx[3]:=RawReference-(Am1+Parameters.Lambda)*FPrefilter[3]-
    (Am2+Am1*Parameters.Lambda)*FPrefilter[2]-
    (Am3+Am2*Parameters.Lambda)*FPrefilter[1]-Am3*Parameters.Lambda*FPrefilter[0];
  SlidingSurface:=Heading+FShunt-FilteredReference;
  Result:=EnsureRange(Parameters.SlidingGain*SlidingSurface+
    Parameters.SwitchingGain*Sign(SlidingSurface),-Parameters.CommandLimit,Parameters.CommandLimit);

  // Identifier uses the PREVIOUS command, before updating the prefilter and shunt.
  Y3:=Heading-Parameters.IdentifierFilter[2]*FFilterY[0]-
    Parameters.IdentifierFilter[1]*FFilterY[1]-Parameters.IdentifierFilter[0]*FFilterY[2];
  U3:=Command-Parameters.IdentifierFilter[2]*FFilterU[0]-
    Parameters.IdentifierFilter[1]*FFilterU[1]-Parameters.IdentifierFilter[0]*FFilterU[2];
  Phi[0]:=FFilterY[2]; Phi[1]:=FFilterY[1]; Phi[2]:=-FFilterU[1]; Phi[3]:=-FFilterU[0];
  Residual:=Y3;
  for I:=0 to 3 do Residual:=Residual+Theta[I]*Phi[I];
  Dy[0]:=FFilterY[1]; Dy[1]:=FFilterY[2]; Dy[2]:=Y3;
  Du[0]:=FFilterU[1]; Du[1]:=FFilterU[2]; Du[2]:=U3;
  for I:=0 to 2 do
  begin FFilterY[I]:=FFilterY[I]+Dt*Dy[I]; FFilterU[I]:=FFilterU[I]+Dt*Du[I]; end;
  if Solve4(FCovariance,Phi,Solved) then
    for I:=0 to 3 do Theta[I]:=Theta[I]-Dt*Solved[I]*Residual;
  for I:=0 to 3 do Theta[I]:=EnsureRange(Theta[I],-Limits[I],Limits[I]);
  for I:=0 to 3 do for J:=I to 3 do
  begin
    Updated:=FCovariance[I,J]+Dt*(Phi[I]*Phi[J]-Parameters.IdentifierAlpha*FCovariance[I,J]);
    FCovariance[I,J]:=Updated; FCovariance[J,I]:=Updated;
  end;
  for I:=0 to 3 do FPrefilter[I]:=FPrefilter[I]+Dt*Dx[I];
  Command:=Result;
  Decay:=Exp(-Parameters.Lambda*Dt);
  if Parameters.Lambda=0 then Input:=Parameters.Kappa*Dt
  else Input:=Parameters.Kappa*(1-Decay)/Parameters.Lambda;
  FShunt:=Decay*FShunt+Input*Command;
end;

function TAdaptiveState.Update(Dt, MeasuredAngleDegrees, GyroDegreesPerSecond: Double): Double;
var Previous:TAdaptiveState; I:Integer;
begin
  if IsNan(Dt) or IsInfinite(Dt) or (Dt<0) or
    IsNan(MeasuredAngleDegrees) or IsInfinite(MeasuredAngleDegrees) or
    IsNan(GyroDegreesPerSecond) or IsInfinite(GyroDegreesPerSecond) then
    raise EArgumentException.Create('Adaptive inputs must be finite; dt must be non-negative');
  if Dt=0 then Exit(Output);
  Previous:=Self;
  try
    // Same sample timing as the planar plant: current gyro describes the preceding interval.
    Heading:=Heading+DegToRad(GyroDegreesPerSecond)*FPreviousDt;
    Output:=RadToDeg(Parameters.RateGain*Calculate(Dt,DegToRad(MeasuredAngleDegrees)));
    FPreviousDt:=Dt;
    if IsNan(Output) or IsInfinite(Output) or IsNan(Residual) or IsInfinite(Residual) then
      raise EMathError.Create('Adaptive controller numerical failure');
    for I:=0 to 3 do
      if IsNan(Theta[I]) or IsInfinite(Theta[I]) or IsNan(FPrefilter[I]) or IsInfinite(FPrefilter[I]) then
        raise EMathError.Create('Adaptive controller state is not finite');
  except
    Self:=Previous;
    raise;
  end;
  Result:=Output;
end;
end.
