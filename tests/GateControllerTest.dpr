program GateControllerTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, uGateNavigationPlus in '..\uGateNavigationPlus.pas';

procedure Check(B: Boolean; const S: string);
begin if not B then raise Exception.Create(S); end;
procedure Near(A, B, E: Double; const S: string);
begin Check(Abs(A-B)<=E, S + Format(': %.9f <> %.9f',[A,B])); end;

procedure ObserverAndTiming;
var P: TGatePlusParameters; C, Other: TGatePlusController; I: Integer;
  T, Dt, PreviousDt, Heading, Expected: Double;
  Angles: array[0..2600] of Double;
begin
  P:=DefaultGatePlusParameters; P.SensorDelay:=1000; C.Reset;
  T:=0; PreviousDt:=0; Heading:=0;
  for I:=0 to High(Angles) do
  begin
    Dt:=0.005+(I mod 3)*0.002;
    Heading:=Heading+20*PreviousDt;
    Angles[I]:=170+10*T-Heading;
    if I>=1000 then C.Update(P,Dt,Angles[I-1000],20,True,False)
    else C.Update(P,Dt,0,20,False,False);
    Expected:=P.ObserverSlipTime*20*(1-Exp(-T/P.ObserverSlipTime));
    Near(C.EstimatedSlip,Expected,1E-9,'Exact observer with preceding variable interval');
    PreviousDt:=Dt; T:=T+Dt;
  end;
  Near(C.LOSRate,10,0.001,'Delay ring wrap with variable dt');
  Near(C.CorrectedAngle,170+10*(T-Dt)-Heading,0.01,'Variable delay prediction');
  P.SensorDelay:=0; C.Reset;
  for I:=1 to 300 do C.Update(P,0.01,0,50,True,False);
  Check(C.EstimatedSlip>19,'Slip built up');
  C.Update(P,0.01,0,0,False,True);
  Check(C.PredictedCourseChange>9,'Zero yaw does not mean straight course');
  Check(C.Mode=gpmPassage,'Terminal loss starts passage plan');
  Other:=C; C.Update(P,0,0,0,False,False);
  Check(CompareMem(@C,@Other,SizeOf(C)),'Zero dt must not mutate');
  C.Reset; P.ObserverSlipTime:=0;
  C.Update(P,0.01,0,80,True,False); C.Update(P,0.01,0,80,True,False);
  Near(C.EstimatedSlip,0,0,'Zero tau');
  Near(C.PredictedCourseChange,0,0,'Zero tau residual');
end;

procedure ModesAndLimits;
var P: TGatePlusParameters; C, Other: TGatePlusController; I: Integer;
  Dt, Old, Limit, Angle: Double;
begin
  P:=DefaultGatePlusParameters; P.SensorDelay:=0; P.KLambda:=0.5; C.Reset;
  for I:=0 to 299 do C.Update(P,0.01,20+I*0.03,10,True,False);
  Check(C.Mode=gpmGuidance,'Normal guidance');
  C.Update(P,0.01,0,10,False,False);
  Check(C.TargetLost and not C.PredictionExpired,'Short loss forecast');
  Check((C.Mode=gpmGuidance) and not C.PassageConfirmed,'Loss is not passage evidence');
  C.Update(P,0.01,30,10,True,True);
  Check((C.Mode=gpmTerminal) and not C.TargetLost,'Terminal acquisition');
  Old:=C.CommandRate;
  for I:=0 to 199 do
  begin
    Dt:=0.005+(I mod 3)*0.002;
    C.Update(P,Dt,NaN,10,False,False);
    Check(C.Mode=gpmPassage,'Passage plan after near-zone loss');
    Check(not C.PassageConfirmed,'Blind plan is not confirmed passage');
    Check(Abs(C.CommandRate-Old)<=P.TerminalCommandAcceleration*Dt+1E-8,'Loss slew');
    Check(Abs(C.Output)<=P.MaxRate,'Actuator limit');
    Check(not IsNan(C.Output) and not IsInfinite(C.Output),'Finite lost-target output');
    Old:=C.CommandRate;
  end;
  Check(C.PredictionExpired,'Forecast expires');
  Check(Abs(C.CommandRate)<=P.SafeRate+1E-8,'Bounded fallback after settling');
  C.Update(P,0.01,-170,10,True,False);
  Check((C.Mode=gpmTerminal) and not C.PredictionExpired,'Reacquire terminal target');
  Near(C.LOSRate,0,0,'Reacquisition no rate impulse');
  Near(C.LOSAcceleration,0,0,'Reacquisition no lead impulse');
  C.Update(P,0.01,0,10,True,False,True);
  Check(C.PassageConfirmed,'Explicit confirmation');
  C.Update(P,0.01,0,10,True,False);
  Check(C.PassageConfirmed and (C.Mode=gpmPassage),'Confirmation latched');
  C.Reset;
  Check((C.Mode=gpmGuidance) and not C.PassageConfirmed and not C.TargetLost,'Reset flags');
  Near(C.EstimatedSlip,0,0,'Reset observer');
  // Adversarial jumps and noise cannot violate the command slew or output bound.
  for I:=0 to 1999 do
  begin
    Dt:=0.004+(I mod 7)*0.001;
    Angle:=179*Cos(I*1.7);
    Old:=C.CommandRate;
    C.Update(P,Dt,Angle,80*Sin(I*0.03),(I mod 17)<>0,I=500);
    Limit:=P.MaxCommandAcceleration;
    if (C.Mode<>gpmGuidance) or C.TargetLost then Limit:=Min(Limit,P.TerminalCommandAcceleration);
    Check(Abs(C.CommandRate-Old)<=Limit*Dt+1E-8,'Noisy command slew');
    Check(Abs(C.CommandRate)<=P.MaxRate,'Noisy command bound');
    Check(Abs(C.Output)<=P.MaxRate,'Noisy actuator bound');
  end;
  Other:=C;
  try C.Update(P,-1,0,0,True,False); Check(False,'Negative dt accepted');
  except on E:EArgumentException do ; end;
  Check(CompareMem(@C,@Other,SizeOf(C)),'Invalid dt changed state');
  try C.Update(P,0.01,0,NaN,True,False); Check(False,'Invalid gyro accepted');
  except on E:EArgumentException do ; end;
  Check(CompareMem(@C,@Other,SizeOf(C)),'Invalid gyro changed state');
end;

procedure SimulatorEvidence;
var M: TGatePlusSimulation; I: Integer;
begin
  M.Parameters:=DefaultGatePlusParameters;
  M.Parameters.SpeedKmh:=36; M.Parameters.SensorDelay:=3;
  M.Parameters.FreezeDistance:=0; M.Reset(100,0);
  M.Step(0.01,100,0,0,0,False);
  for I:=1 to 3 do begin
    M.Step(0.01,100,0,0,0);
    Check(M.Controller.TargetLost,'Invalid sample follows detector delay');
  end;
  M.Step(0.01,100,0,0,0);
  Check(not M.Controller.TargetLost,'Delayed detector reacquisition');
  M.Parameters.DetectorHalfAngle:=30; M.Reset(0,100);
  for I:=1 to 10 do M.Step(0.01,0,100,0,0);
  Check(M.Controller.TargetLost,'Outside field of view');
  Check(not M.Controller.PassageConfirmed,'Field edge is not a passage');
  M.Parameters:=DefaultGatePlusParameters; M.Parameters.InitialHeadingDegrees:=180;
  M.Parameters.SensorDelay:=0; M.Reset(1,0);
  M.Step(0.01,1,0,0,0);
  Check(M.Controller.Mode=gpmGuidance,'Nearby receding target is not terminal');
  M.Parameters:=DefaultGatePlusParameters; M.Parameters.InitialSlipDegrees:=20;
  M.Reset(100,0); Near(M.Heading-M.Course,20,1E-9,'Initial plant slip');
  Near(M.Controller.EstimatedSlip,0,0,'True slip not leaked into observer');
  M.Parameters.SpeedKmh:=0; M.Reset(1,0);
  for I:=1 to 50 do M.Step(0.01,1,0,0,0);
  Near(M.X,0,0,'Stationary plant');
  Check(M.Controller.Mode=gpmGuidance,'No false closing evidence at zero speed');
  M.Parameters:=DefaultGatePlusParameters; M.Reset(100,0);
  for I:=1 to 300 do M.Step(0.01,100,0,0,0);
  Check(M.Controller.PassageConfirmed,'Closest approach detected');
  Near(M.MinimumDistance,0,1E-8,'Segment minimum');
end;

begin
  try
    ObserverAndTiming; ModesAndLimits; SimulatorEvidence;
    Writeln('PASS: exact slip observer, variable dt/ring wrap, residual course rotation, modes,');
    Writeln('      dropout/reacquisition, noise/slew, limits, invalid inputs, simulator-only evidence');
  except on E:Exception do begin Writeln(E.ClassName,': ',E.Message); ExitCode:=1; end; end;
end.