program BiasAdaptationTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math,
  uMotionModel in '..\uMotionModel.pas',
  uPID in '..\uPID.pas';
const
  Angles: array[0..5] of Double=(-65,-35,-5,5,35,65);
  Biases: array[0..4] of Double=(-30,-10,0,10,30);
  VariableDt: array[0..3] of Double=(0.007,0.01,0.009,0.008);
var
  M: TMotionModel;
  C,A,B: TPIDState;
  Pattern,R,Dir,Bias,I,N: Integer;
  Dt,Dist,Previous,Nearest,SumMinimumDistance,MaxMinimumDistance,OldBias: Double;
  SumIntegral,SumW,Weight,Time,LateTime: Double;
begin
  try
    for Pattern:=0 to 2 do
    begin
      N:=0; SumMinimumDistance:=0; MaxMinimumDistance:=0; SumIntegral:=0; SumW:=0; Weight:=0;
      for R:=0 to 1 do for Dir:=0 to High(Angles) do for Bias:=0 to High(Biases) do
      begin
        M.Init; PIDinit(C);
        M.ReferencePoint.V := 0; // Исходные 180 сценариев с неподвижной точкой.
        M.ReferencePoint.X:=(120+100*R)*Cos(DegToRad(Angles[Dir]));
        M.ReferencePoint.Y:=(120+100*R)*Sin(DegToRad(Angles[Dir]));
        M.SensorBiasDegrees:=Biases[Bias];
        Dist:=Hypot(M.ReferencePoint.X,M.ReferencePoint.Y); Nearest:=Dist;
        LateTime:=0.65*Dist/M.V; Time:=0;
        for I:=0 to 1799 do
        begin
          case Pattern of
            0: Dt:=0.005;
            1: Dt:=0.01;
            else Dt:=VariableDt[I mod 4];
          end;
          Previous:=Dist; OldBias:=C.AdaptiveAngleCorrection;
          M.W:=DegToRad(PIDupdate(C, Dt,RadToDeg(M.MeasureAngle),RadToDeg(M.W)));
          if Abs(C.AdaptiveAngleCorrection-OldBias)>BiasRateLimit*Dt+1E-10 then
            raise Exception.Create('Bias rate limit');
          if Abs(C.AdaptiveAngleCorrection)>BiasLimit+1E-10 then
            raise Exception.Create('Bias magnitude limit');
          M.Integrate(Dt);
          Dist:=Hypot(M.ReferencePoint.X-M.X,M.ReferencePoint.Y-M.Y);
          Nearest:=Min(Nearest,Dist);
          if Time>LateTime then
          begin
            SumIntegral:=SumIntegral+Abs(C.Wint)*Dt;
            SumW:=SumW+Abs(RadToDeg(M.W))*Dt; Weight:=Weight+Dt;
          end;
          Time:=Time+Dt;
          if Dist>Previous+1E-9 then Break;
        end;
        if I>=1799 then raise Exception.Create('Simulation did not stop');
        if IsNan(Nearest) or (Nearest>0.35) then
          raise Exception.CreateFmt('Distance result outside test tolerance: pattern %d range %d angle %d bias %d: %.6f',
            [Pattern,R,Dir,Bias,Nearest]);
        Inc(N); SumMinimumDistance:=SumMinimumDistance+Nearest; MaxMinimumDistance:=Max(MaxMinimumDistance,Nearest);
      end;
      Writeln(Format('Timing %d: %d cases; mean/max minimum distance %.6f / %.6f m; late |I|/|W| %.6f / %.6f deg/s',
        [Pattern,N,SumMinimumDistance/N,MaxMinimumDistance,SumIntegral/Weight,SumW/Weight]));
    end;
    PIDinit(A); PIDinit(B);
    for I:=1 to 110 do
    begin
      PIDupdate(A, 0.01,0,0); PIDupdate(B, 0.01,0,0.1);
    end;
    if Abs(A.AdaptiveAngleCorrection-B.AdaptiveAngleCorrection)<1E-6 then
      raise Exception.Create('Input W is ignored');
    B.AngleCorrectionEnabled:=False; OldBias:=B.AdaptiveAngleCorrection;
    for I:=1 to 20 do PIDupdate(B, 0.01,0.2,0.1);
    if B.AdaptiveAngleCorrection<>OldBias then raise Exception.Create('Disabled adaptation changed bias');
    if Abs(B.CorrectedAngle-0.2)>1E-10 then raise Exception.Create('Disabled correction applied');
    PIDinit(B);
    if (B.AdaptiveAngleCorrection<>0) or (B.Wint<>0) or (B.Wdiff<>0) then
      raise Exception.Create('Reset state');
    Writeln('PASS: 180 holdout cases, real W feedback, adaptation limits, disable/reset');
  except on E: Exception do begin Writeln(E.Message); ExitCode:=1; end; end;
end.
