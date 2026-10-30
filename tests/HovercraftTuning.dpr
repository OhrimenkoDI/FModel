program HovercraftTuning;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, System.IniFiles,
 uRecordedPlant, uPID, uGateNavigation, uGateNavigationPlus, uAdaptiveGlide;
type
 TSetup=record P:TPIDParameters;G:TGateParameters;Q:TGatePlusParameters;A:TAdaptiveParameters;end;
 TMetric=record Cost,RMS,Tail,Sat,Peak,IAE:Double;end;
var OutDir:string; Detail:TextFile; Kind:Integer;
function Bearing(Mode:Integer;T:Double):Double;
begin
 case Mode of
  0:Result:=5;1:Result:=15;2:Result:=-30;
  3:Result:=8*Sin(T*0.8);
  4:if T<2 then Result:=0 else if T<4 then Result:=12 else Result:=-8;
  5:Result:=2*T;
  6:Result:=10*Sin(T*1.3);
  else Result:=-10;
 end;
end;
function Run(const S:TSetup;Mode,Stress,Seed:Integer;Duration:Double;Trace:string=''):TMetric;
var Plant:TRecordedPlant;P:TPIDState;G:TGateController;Q:TGatePlusController;A:TAdaptiveState;
 T,H,E,U,Prev,TV,Scale,Measured,TailSum:Double;I,Count,NTail:Integer;
 F:TextFile;
begin
 Result:=Default(TMetric);Plant:=Default(TRecordedPlant);Plant.Parameters:=DefaultRecordedPlantParameters;
 Plant.Parameters.Profile:=3;Plant.Parameters.CameraHz:=50;Plant.Parameters.CameraDelay:=0.04;
 Plant.Parameters.CompensationTime:=0;Plant.Parameters.Seed:=Seed;Plant.Parameters.OutputLimit:=40;
 case Stress of
  1:begin Plant.Parameters.GainScale:=1.25;Plant.Parameters.ExtraDelay:=0.03;Plant.Parameters.FrequencyScale:=0.85;Plant.Parameters.NoiseScale:=1.5;end;
  2:begin Plant.Parameters.GainScale:=0.75;Plant.Parameters.FrequencyScale:=1.15;Plant.Parameters.CameraDelay:=0.06;Plant.Parameters.NoiseScale:=2;end;
  3:begin Plant.Parameters.CameraBias:=7;Plant.Parameters.CourseTime:=0.7;end;
  4:begin Plant.Parameters.CameraBias:=-2;Plant.Parameters.CameraDelay:=0.02;end;
 end;
 Plant.Reset;PIDinit(P);P.Parameters:=S.P;P.AngleCorrectionEnabled:=True;PIDreset(P);
 G.Reset;Q.Reset;A.Init;A.Parameters:=S.A;A.Reset;
 H:=0;Prev:=0;TV:=0;TailSum:=0;NTail:=0;Count:=Round(Duration/0.01);Scale:=Max(5,Abs(Bearing(Mode,0)));
 if Mode in [3,4,5,6] then Scale:=10;
 if Trace<>'' then begin AssignFile(F,OutDir+Trace);Rewrite(F);Writeln(F,'time,error,measured,rate,command');end;
 for I:=0 to Count-1 do begin
  T:=I*0.01;E:=Bearing(Mode,T)-H;Measured:=Plant.Observe(E);
  case Kind of
   0:U:=PIDupdate(P,0.01,Measured,Plant.GyroRate);
   1:U:=G.Update(S.G,0.01,Measured,Plant.GyroRate,True,False);
   2:U:=A.Update(0.01,Measured,Plant.GyroRate);
   else U:=Q.Update(S.Q,0.01,Measured,Plant.GyroRate,True,False);
  end;
  if IsNan(U) or IsInfinite(U) or (Abs(E)>170) then begin Result.Cost:=1E6;if Trace<>'' then CloseFile(F);Exit;end;
  U:=EnsureRange(U,-40.0,40.0);if Abs(U)>39.9 then Result.Sat:=Result.Sat+1;
  Result.IAE:=Result.IAE+Abs(E)*0.01;Result.RMS:=Result.RMS+Sqr(E);Result.Peak:=Max(Result.Peak,Abs(E));
  TV:=TV+Abs(U-Prev);Prev:=U;
  if T>=Duration-1.5 then begin TailSum:=TailSum+Sqr(E);Inc(NTail);end;
  if Trace<>'' then Writeln(F,Format('%.3f,%.6f,%.6f,%.6f,%.6f',[T,E,Measured,Plant.Rate,U]));
  Plant.Advance(U,0.01);H:=H+Plant.Rate*0.01;
 end;
 if Trace<>'' then CloseFile(F);
 Result.RMS:=Sqrt(Result.RMS/Count);Result.Tail:=Sqrt(TailSum/NTail);Result.Sat:=Result.Sat/Count;
 Result.Cost:=Result.IAE/(Duration*Scale)+Result.Tail/Scale+0.15*Result.Sat+0.0008*TV/Duration+0.3*Max(0,Result.Peak/Scale-1);
end;
function Score(const S:TSetup):Double;
var M,V:Integer;R:TMetric;Sum,Worst:Double;
begin
 Sum:=0;Worst:=0;
 for M:=0 to 3 do for V:=0 to 1 do begin R:=Run(S,M,V,20261029,6);Sum:=Sum+R.Cost;Worst:=Max(Worst,R.Cost);end;
 Result:=Sum/8+0.25*Worst;
end;
function LU(L,H:Double):Double;
begin Result:=Exp(Ln(L)+Random*(Ln(H)-Ln(L)));end;
function Start:TSetup;
begin
 Result.P:=PIDDefaultParameters;Result.P.Kp:=12;Result.P.Ki:=20;Result.P.Kd:=3;Result.P.IntegralLimit:=50;
 Result.P.BiasAdaptationGain:=0.1;Result.P.BiasLimit:=20;
 Result.G:=DefaultGateParameters;Result.G.N:=3;Result.G.LeadTime:=0;Result.G.KLambda:=3;Result.G.MaxRate:=40;
 Result.G.FilterTime:=0.15;Result.G.RateKp:=0.35;Result.G.RateKi:=3;Result.G.SensorDelay:=4;
 Result.Q:=DefaultGatePlusParameters;Result.Q.MaxRate:=40;
 Result.A:=DefaultAdaptiveParameters;
end;
procedure MatchPlus(var S:TSetup);
begin
 S.Q.N:=S.G.N;S.Q.LeadTime:=S.G.LeadTime;S.Q.KLambda:=S.G.KLambda;S.Q.MaxRate:=40;
 S.Q.FilterTime:=S.G.FilterTime;S.Q.RateKp:=S.G.RateKp;S.Q.RateKi:=S.G.RateKi;S.Q.SensorDelay:=S.G.SensorDelay;
end;
procedure Save(const S:TSetup;const Name:string);
var Ini:TMemIniFile;
 procedure V(const Section,Key:string;X:Double);begin Ini.WriteFloat(Section,Key,X);end;
begin
 Ini:=TMemIniFile.Create(OutDir+Name+'.ini');try
 V('Controller','Kp',S.P.Kp);V('Controller','Ki',S.P.Ki);V('Controller','Kd',S.P.Kd);
 V('Controller','IntegralLimitDegreesPerSecond',S.P.IntegralLimit);V('Controller','BiasAdaptationGain',S.P.BiasAdaptationGain);
 V('Controller','DerivativeFilterTime',S.P.DerivativeFilterTime);
 V('GateNavigation','N',S.G.N);V('GateNavigation','LeadTime',S.G.LeadTime);V('GateNavigation','KLambda',S.G.KLambda);
 V('GateNavigation','MaxRate',40);V('GateNavigation','FilterTime',S.G.FilterTime);V('GateNavigation','RateKp',S.G.RateKp);
 V('GateNavigation','RateKi',S.G.RateKi);V('GateNavigation','SensorDelay',S.G.SensorDelay);
 V('GateNavigationPlus','N',S.Q.N);V('GateNavigationPlus','LeadTime',S.Q.LeadTime);V('GateNavigationPlus','KLambda',S.Q.KLambda);
 V('GateNavigationPlus','MaxRate',40);V('GateNavigationPlus','FilterTime',S.Q.FilterTime);V('GateNavigationPlus','RateKp',S.Q.RateKp);
 V('GateNavigationPlus','RateKi',S.Q.RateKi);V('GateNavigationPlus','SensorDelay',S.Q.SensorDelay);
 V('GateNavigationPlus','MaxCommandAcceleration',S.Q.MaxCommandAcceleration);V('GateNavigationPlus','LeadFilterTime',S.Q.LeadFilterTime);
 V('AdaptiveGlide','RateGain',S.A.RateGain);V('AdaptiveGlide','SlidingGain',S.A.SlidingGain);V('AdaptiveGlide','SwitchingGain',S.A.SwitchingGain);
 V('AdaptiveGlide','Kappa',S.A.Kappa);V('AdaptiveGlide','Lambda',S.A.Lambda);V('AdaptiveGlide','CovarianceScale',S.A.CovarianceScale);
 Ini.UpdateFile;finally Ini.Free;end;
end;
procedure Report(const S:TSetup;Name:string);
var M,V,Seed:Integer;R:TMetric;Trace:string;
begin
 for M:=0 to 7 do for V:=0 to 4 do for Seed:=1 to 2 do begin
  Trace:='';if (M=1) and (V=0) and (Seed=1) then Trace:=Format('trace_%d_%s.csv',[Kind,Name]);
  R:=Run(S,M,V,20262000+Seed,8,Trace);
  Writeln(Detail,Format('%d,%s,%d,%d,%d,%.7f,%.7f,%.7f,%.7f,%.7f',[Kind,Name,M,V,Seed,R.Cost,R.RMS,R.Tail,R.Sat,R.Peak]));
 end;
end;
var Baseline,Best,Candidate,Combined:TSetup;BestScore,C:Double;I,J:Integer;
begin
 try
 FormatSettings.DecimalSeparator:='.';RandSeed:=20261029;
 OutDir:=IncludeTrailingPathDelimiter(ExpandFileName(ParamStr(1)));ForceDirectories(OutDir);
 Combined:=Start;AssignFile(Detail,OutDir+'validation.csv');Rewrite(Detail);Writeln(Detail,'kind,setting,mode,stress,seed,cost,rms,tail,saturation,peak_error');
 for Kind:=0 to 3 do begin
  Baseline:=Start;Best:=Baseline;
  if Kind=3 then begin Best.G:=Combined.G;MatchPlus(Best);end;
  BestScore:=Score(Best);
  for I:=1 to 500 do begin
   Candidate:=Best;
   case Kind of
   0:begin
    if I mod 3=0 then begin Candidate.P.Kp:=LU(0.3,8);Candidate.P.Ki:=LU(0.02,6);Candidate.P.Kd:=LU(0.02,3);Candidate.P.DerivativeFilterTime:=LU(0.015,0.15);end
    else case Random(4) of
     0:Candidate.P.Kp:=EnsureRange(Best.P.Kp*LU(0.7,1.4),0.1,12.0);
     1:Candidate.P.Ki:=EnsureRange(Best.P.Ki*LU(0.6,1.5),0.005,10.0);
     2:Candidate.P.Kd:=EnsureRange(Best.P.Kd*LU(0.7,1.4),0.0,5.0);
     3:Candidate.P.DerivativeFilterTime:=EnsureRange(Best.P.DerivativeFilterTime*LU(0.7,1.4),0.01,0.25);
    end;
    Candidate.P.IntegralLimit:=20;Candidate.P.BiasAdaptationGain:=0;
   end;
   1,3:begin
    if I mod 3=0 then begin
     Candidate.G.N:=Random*2;Candidate.G.KLambda:=LU(0.3,5);Candidate.G.RateKp:=LU(0.05,1);Candidate.G.RateKi:=LU(0.1,5);
     Candidate.G.FilterTime:=LU(0.04,0.3);Candidate.G.SensorDelay:=Random(7);
    end else case Random(6) of
     0:Candidate.G.N:=EnsureRange(Best.G.N+(Random-0.5)*0.6,0.0,3.0);
     1:Candidate.G.KLambda:=EnsureRange(Best.G.KLambda*LU(0.7,1.4),0.2,6.0);
     2:Candidate.G.RateKp:=EnsureRange(Best.G.RateKp*LU(0.7,1.4),0.02,1.5);
     3:Candidate.G.RateKi:=EnsureRange(Best.G.RateKi*LU(0.7,1.4),0.02,8.0);
     4:Candidate.G.FilterTime:=EnsureRange(Best.G.FilterTime*LU(0.7,1.4),0.02,0.4);
     5:Candidate.G.SensorDelay:=Random(7);
    end;
    Candidate.G.MaxRate:=40;Candidate.G.LeadTime:=0;
    if Kind=3 then begin MatchPlus(Candidate);Candidate.Q.MaxCommandAcceleration:=LU(30,360);Candidate.Q.LeadFilterTime:=LU(0.05,0.3);end;
   end;
   2:begin
    if I mod 3=0 then begin Candidate.A.RateGain:=-LU(0.4,8);Candidate.A.SlidingGain:=LU(1,30);Candidate.A.Kappa:=-LU(0.02,0.5);Candidate.A.Lambda:=LU(0.3,4);end
    else case Random(4) of
     0:Candidate.A.RateGain:=-EnsureRange(Abs(Best.A.RateGain)*LU(0.7,1.4),0.2,10.0);
     1:Candidate.A.SlidingGain:=EnsureRange(Best.A.SlidingGain*LU(0.7,1.4),0.3,60.0);
     2:Candidate.A.Kappa:=-EnsureRange(Abs(Best.A.Kappa)*LU(0.7,1.4),0.005,1.0);
     3:Candidate.A.Lambda:=EnsureRange(Best.A.Lambda*LU(0.7,1.4),0.1,8.0);
    end;
   end;
   end;
   C:=Score(Candidate);if C<BestScore then begin Best:=Candidate;BestScore:=C;end;
   if I mod 100=0 then Writeln('kind=',Kind,' iteration=',I,' cost=',BestScore:0:5);
  end;
  case Kind of 0:Combined.P:=Best.P;1:Combined.G:=Best.G;2:Combined.A:=Best.A;3:Combined.Q:=Best.Q;end;
  Save(Best,'best_'+IntToStr(Kind));Report(Baseline,'baseline');Report(Best,'tuned');
 end;
 Save(Combined,'combined');CloseFile(Detail);Writeln('DONE');
 except on E:Exception do begin Writeln(E.ClassName,': ',E.Message);ExitCode:=1;end;end;
end.
