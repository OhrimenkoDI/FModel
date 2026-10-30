program AdaptiveTuning;
{$APPTYPE CONSOLE}
uses System.SysUtils,System.Math,System.IniFiles,uAdaptiveGlide in '..\uAdaptiveGlide.pas',uMotionModel in '..\uMotionModel.pas';
type
 TScenario=record X,Y,Speed,Heading,TargetSpeed,TargetHeading,Dt:Double; Delay:Integer; end;
 TMetrics=record Miss,Time,Variation:Double; Failed:Boolean; end;
const TrainCount=24; TotalCount=32;
var Scenarios:array[0..TotalCount-1]of TScenario; Base,Best,Candidate:TAdaptiveParameters;
 BestScore,Score,Band:Double; Iter,I,J:Integer; Metrics:TMetrics; OutFile:TextFile; Ini:TMemIniFile;
function Run(const P:TAdaptiveParameters; const S:TScenario):TMetrics;
var M:TMotionModel; C:TAdaptiveState; I:Integer; Dt,T,D,Before,OldW:Double; Approached:Boolean;
begin
 Result.Miss:=1E6; Result.Time:=30; Result.Variation:=0; Result.Failed:=False;
 M.Init; M.SpeedKmh:=S.Speed; M.InitialHeadingDegrees:=S.Heading;
 M.SensorBiasDegrees:=0; M.AngleCount:=S.Delay;
 M.ReferencePoint.InitialX:=S.X; M.ReferencePoint.InitialY:=S.Y;
 M.ReferencePoint.SpeedKmh:=S.TargetSpeed; M.ReferencePoint.HeadingDegrees:=S.TargetHeading; M.ResetMotion;
 C.Init; C.Parameters:=P; C.Reset; T:=0; OldW:=0; Approached:=False;
 Before:=Hypot(M.ReferencePoint.X-M.X,M.ReferencePoint.Y-M.Y);
 Result.Miss:=Before;
 try
 for I:=0 to 10000 do begin
  Dt:=S.Dt; if Dt=0 then Dt:=0.005+(I mod 3)*0.002;
  M.W:=DegToRad(C.Update(Dt,RadToDeg(M.MeasureAngle),RadToDeg(M.W)));
  M.Integrate(Dt); T:=T+Dt;
  D:=Hypot(M.ReferencePoint.X-M.X,M.ReferencePoint.Y-M.Y);
  Result.Variation:=Result.Variation+Abs(M.W-OldW); OldW:=M.W;
  if D<Result.Miss then begin Result.Miss:=D; Result.Time:=T; end;
  if (D<50) and (D<Before) then Approached:=True;
  if Approached and (D>Before+1E-9) then Break;
  Before:=D;
  if T>=30 then Break;
 end;
 except Result.Failed:=True; Result.Miss:=1E6; end;
 Result.Variation:=Result.Variation/Max(T,0.001);
end;
function Evaluate(const P:TAdaptiveParameters):Double;
var K:Integer; M:TMetrics;
begin
 Result:=0;
 for K:=0 to TrainCount-1 do begin
  M:=Run(P,Scenarios[K]);
  Result:=Result+Min(M.Miss,500)*2+Sqr(Max(0,Min(M.Miss,500)-1))*10+M.Time*0.3+Min(M.Variation,1000)*0.002;
  if M.Miss>=5 then Result:=Result+100;
 end;
end;
procedure WriteParams(const P:TAdaptiveParameters; const FileName:string);
var F:TMemIniFile;
 procedure Put(const K:string; V:Double);
 begin F.WriteString('AdaptiveGlide',K,FloatToStr(V,TFormatSettings.Invariant)); end;
begin
 F:=TMemIniFile.Create(FileName);
 try
 Put('CommandLimit',P.CommandLimit); Put('SlidingGain',P.SlidingGain); Put('SwitchingGain',P.SwitchingGain);
 Put('Kappa',P.Kappa); Put('Lambda',P.Lambda); Put('IdentifierAlpha',P.IdentifierAlpha);
 Put('CovarianceScale',P.CovarianceScale); Put('DenominatorGuard',P.DenominatorGuard);
 Put('RateGain',P.RateGain); Put('AngleLimit',P.AngleLimit);
 Put('Reference1',P.ReferenceCoefficients[0]); Put('Reference2',P.ReferenceCoefficients[1]); Put('Reference3',P.ReferenceCoefficients[2]);
 Put('Identifier1',P.IdentifierFilter[0]); Put('Identifier2',P.IdentifierFilter[1]); Put('Identifier3',P.IdentifierFilter[2]);
 F.UpdateFile;
 finally F.Free; end;
end;
procedure ReadParams(var P:TAdaptiveParameters; const Name:string);
var F:TMemIniFile;
begin
 F:=TMemIniFile.Create(Name);
 try
 P.CommandLimit:=F.ReadFloat('AdaptiveGlide','CommandLimit',P.CommandLimit);
 P.SlidingGain:=F.ReadFloat('AdaptiveGlide','SlidingGain',P.SlidingGain);
 P.SwitchingGain:=F.ReadFloat('AdaptiveGlide','SwitchingGain',P.SwitchingGain);
 P.Kappa:=F.ReadFloat('AdaptiveGlide','Kappa',P.Kappa);
 P.Lambda:=F.ReadFloat('AdaptiveGlide','Lambda',P.Lambda);
 P.IdentifierAlpha:=F.ReadFloat('AdaptiveGlide','IdentifierAlpha',P.IdentifierAlpha);
 P.CovarianceScale:=F.ReadFloat('AdaptiveGlide','CovarianceScale',P.CovarianceScale);
 P.DenominatorGuard:=F.ReadFloat('AdaptiveGlide','DenominatorGuard',P.DenominatorGuard);
 P.RateGain:=F.ReadFloat('AdaptiveGlide','RateGain',P.RateGain);
 P.AngleLimit:=F.ReadFloat('AdaptiveGlide','AngleLimit',P.AngleLimit);
 P.ReferenceCoefficients[0]:=F.ReadFloat('AdaptiveGlide','Reference1',P.ReferenceCoefficients[0]);
 P.ReferenceCoefficients[1]:=F.ReadFloat('AdaptiveGlide','Reference2',P.ReferenceCoefficients[1]);
 P.ReferenceCoefficients[2]:=F.ReadFloat('AdaptiveGlide','Reference3',P.ReferenceCoefficients[2]);
 P.IdentifierFilter[0]:=F.ReadFloat('AdaptiveGlide','Identifier1',P.IdentifierFilter[0]);
 P.IdentifierFilter[1]:=F.ReadFloat('AdaptiveGlide','Identifier2',P.IdentifierFilter[1]);
 P.IdentifierFilter[2]:=F.ReadFloat('AdaptiveGlide','Identifier3',P.IdentifierFilter[2]);
 finally F.Free; end;
end;
function LogRandom(Low,High:Double):Double;
begin Result:=Exp(Ln(Low)+Random*(Ln(High)-Ln(Low))); end;
begin
 FormatSettings.DecimalSeparator:='.'; RandSeed:=1729;
 Base:=DefaultAdaptiveParameters;
 ReadParams(Base,'profiles\AdaptiveGlide-before-tuning.ini');
 Ini:=TMemIniFile.Create('Win32\Debug\FModel.ini');
 try
 for I:=0 to TotalCount-1 do begin
  Scenarios[I].X:=Ini.ReadFloat('ReferencePoint','X',-199.459356757);
  Scenarios[I].Y:=Ini.ReadFloat('ReferencePoint','Y',100.61306639966);
  Scenarios[I].Speed:=150; Scenarios[I].Heading:=0; Scenarios[I].TargetSpeed:=50;
  Scenarios[I].TargetHeading:=-10; Scenarios[I].Dt:=0.01; Scenarios[I].Delay:=3;
 end;
 finally Ini.Free; end;
 Scenarios[1].Y:=80; Scenarios[2].Y:=120;
 Scenarios[3].X:=100; Scenarios[3].Y:=100;
 Scenarios[4].X:=100; Scenarios[4].Y:=-100;
 Scenarios[5].Speed:=120; Scenarios[6].Speed:=180;
 Scenarios[7].Dt:=0.005; Scenarios[8].Dt:=0;
 Scenarios[9].Heading:=-15; Scenarios[10].Heading:=15;
 Scenarios[11].X:=-170; Scenarios[11].Y:=90;
 Scenarios[12].X:=-230; Scenarios[12].Y:=110;
 Scenarios[13].Delay:=0; Scenarios[14].Delay:=6;
 Scenarios[15].TargetSpeed:=0; Scenarios[16].TargetSpeed:=70;
 Scenarios[17].Speed:=240; Scenarios[18].Heading:=90;
 Scenarios[19].X:=100; Scenarios[19].Y:=100; Scenarios[19].Dt:=0.005;
 Scenarios[20].X:=-190; Scenarios[20].Y:=130; Scenarios[20].Dt:=0.006;
 Scenarios[21].X:=-210; Scenarios[21].Y:=70; Scenarios[21].Speed:=160;
 Scenarios[22].Heading:=-8; Scenarios[22].Delay:=5;
 Scenarios[23].Heading:=8; Scenarios[23].TargetSpeed:=60; Scenarios[23].Dt:=0;
 for I:=24 to 31 do begin
  Scenarios[I].X:=-180-(I-24)*7;
  Scenarios[I].Y:=75+(I-24)*8;
  Scenarios[I].Heading:=-12+(I-24)*3;
  Scenarios[I].Speed:=120+(I-24)*10;
  Scenarios[I].Dt:=0.004+(I mod 4)*0.002;
 end;
 Best:=Base; BestScore:=Evaluate(Base); Writeln('Baseline score ',BestScore:0:4);
 if (ParamCount>=2) then begin ReadParams(Best,ParamStr(2)); BestScore:=Evaluate(Best); end;
 if (ParamCount>0) and (ParamStr(1)='--search') then
 for Iter:=0 to 2499 do begin
  Candidate:=Base;
  Candidate.SlidingGain:=LogRandom(0.05,10);
  Candidate.SwitchingGain:=LogRandom(0.0001,0.2);
  Candidate.Kappa:=-LogRandom(0.1,8);
  Candidate.Lambda:=LogRandom(2,25);
  Candidate.IdentifierAlpha:=LogRandom(0.01,5);
  Candidate.CovarianceScale:=LogRandom(0.01,1000);
  Candidate.DenominatorGuard:=LogRandom(0.2,3);
    Candidate.AngleLimit:=0.3+Random*2.8;
  Band:=LogRandom(0.5,2);
  Candidate.ReferenceCoefficients[0]:=14.2*Band;
  Candidate.ReferenceCoefficients[1]:=51*Sqr(Band);
  Candidate.ReferenceCoefficients[2]:=90*Band*Sqr(Band);
  if (Iter>50) and (Iter mod 2=0) then begin
    Candidate:=Best;
    case Random(9) of
      0: Candidate.SlidingGain:=EnsureRange(Best.SlidingGain*LogRandom(0.5,2),0.01,20);
      1: Candidate.SwitchingGain:=EnsureRange(Best.SwitchingGain*LogRandom(0.3,3),0.00001,0.3);
      2: Candidate.Kappa:=EnsureRange(Best.Kappa*LogRandom(0.5,2),-20.0,-0.01);
      3: Candidate.Lambda:=EnsureRange(Best.Lambda*LogRandom(0.5,2),0.1,40.0);
      4: Candidate.IdentifierAlpha:=EnsureRange(Best.IdentifierAlpha*LogRandom(0.3,3),0.001,10.0);
      5: Candidate.CovarianceScale:=EnsureRange(Best.CovarianceScale*LogRandom(0.1,10),0.001,10000.0);
      6: Candidate.DenominatorGuard:=EnsureRange(Best.DenominatorGuard*LogRandom(0.5,2),0.01,10.0);
      7: Candidate.AngleLimit:=EnsureRange(Best.AngleLimit*LogRandom(0.7,1.4),0.1,Pi);
      8: begin
        Band:=LogRandom(0.8,1.25);
        Candidate.ReferenceCoefficients[0]:=Best.ReferenceCoefficients[0]*Band;
        Candidate.ReferenceCoefficients[1]:=Best.ReferenceCoefficients[1]*Sqr(Band);
        Candidate.ReferenceCoefficients[2]:=Best.ReferenceCoefficients[2]*Band*Sqr(Band);
      end;
    end;
  end;
  // Preserve the current physical rate bound and all filter polynomials.
  Score:=Evaluate(Candidate);
  if Score<BestScore then begin
   Best:=Candidate; BestScore:=Score; Writeln('Best ',Iter,': ',Score:0:4);
   WriteParams(Best,'Win32\GateTests\adaptive-tuned.ini');
  end;
 end;
 AssignFile(OutFile,'Win32\GateTests\adaptive-tuning.csv'); Rewrite(OutFile);
 Writeln(OutFile,'scenario,set,miss_m,time_s,command_variation_rad_s2,x_m,y_m,speed_kmh,heading_deg,target_speed_kmh,target_heading_deg,dt_s,delay_samples');
 for I:=0 to TotalCount-1 do for J:=0 to 1 do begin
  if J=0 then Metrics:=Run(Base,Scenarios[I]) else Metrics:=Run(Best,Scenarios[I]);
  Writeln(OutFile,I,',',J,',',Metrics.Miss:0:6,',',Metrics.Time:0:4,',',Metrics.Variation:0:4,
    ',',Scenarios[I].X:0:9,',',Scenarios[I].Y:0:9,',',Scenarios[I].Speed:0:3,
    ',',Scenarios[I].Heading:0:3,',',Scenarios[I].TargetSpeed:0:3,',',Scenarios[I].TargetHeading:0:3,
    ',',Scenarios[I].Dt:0:6,',',Scenarios[I].Delay);
 end;
 CloseFile(OutFile); WriteParams(Best,'Win32\GateTests\adaptive-tuned.ini');
 Writeln('Final score ',BestScore:0:4);
end.
