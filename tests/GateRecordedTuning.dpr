program GateRecordedTuning;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, System.IniFiles,
  uRecordedPlant in '..\uRecordedPlant.pas',
  uGateNavigation in '..\uGateNavigation.pas';
type
  TMetrics = record Cost, RMS, TailRMS, IAE, Sat, PeakRate, PeakError, Settle: Double; end;
var OutputDir: string; Detail: TextFile;

function Bearing(Mode: Integer; T: Double): Double;
begin
  case Mode of
    0: Result:=1;
    1: Result:=10;
    2: Result:=25;
    3: Result:=5+8*Sin(1.2*T);
    4: if T<2 then Result:=0 else if T<5 then Result:=15 else Result:=-5;
    5: Result:=3*T;
    else Result:=-12;
  end;
end;

function Run(const GP:TGateParameters; Axis,Mode,Stress,Seed:Integer; Duration:Double; Trace:Boolean=False):TMetrics;
var P:TRecordedPlant; G:TGateController; Heading,T,E,U,PreviousU,Scale,TV,Tail,Dt:Double;
    I,Count,TailCount,SatCount:Integer; F:TextFile;
begin
  Result:=Default(TMetrics);P:=Default(TRecordedPlant);
  P.Parameters:=DefaultRecordedPlantParameters;P.Parameters.Profile:=Axis;
  P.Parameters.Seed:=Seed;Dt:=0.01;
  case Stress of
    1: begin P.Parameters.ExtraDelay:=0.02;P.Parameters.GainScale:=1.2;
      P.Parameters.FrequencyScale:=0.85;P.Parameters.CameraDelay:=0.05;P.Parameters.NoiseScale:=1.5;end;
    2: begin P.Parameters.GainScale:=0.75;P.Parameters.FrequencyScale:=0.8;
      P.Parameters.CameraDelay:=0.02;P.Parameters.NoiseScale:=2;end;
    3: begin P.Parameters.GainScale:=1.35;P.Parameters.ExtraDelay:=0.03;
      P.Parameters.DampingScale:=0.8;P.Parameters.CameraDelay:=0.06;P.Parameters.NoiseScale:=2;end;
    4: begin P.Parameters.CameraDelay:=0;P.Parameters.NoiseScale:=0;end;
    5: begin P.Parameters.CameraDelay:=0.07;P.Parameters.CompensationTime:=0.035;P.Parameters.NoiseScale:=2;Dt:=0.005;end;
  end;
  P.Reset;G.Reset;Heading:=0;PreviousU:=0;TV:=0;Tail:=0;TailCount:=0;SatCount:=0;
  Count:=Round(Duration/Dt);Scale:=Max(5,Abs(Bearing(Mode,0)));Result.Settle:=0;
  if Trace then begin AssignFile(F,OutputDir+'trace_'+IntToStr(Axis)+'.csv');Rewrite(F);
    Writeln(F,'time,bearing,heading,error,gyro,command');end;
  for I:=0 to Count-1 do begin
    T:=I*Dt;E:=Bearing(Mode,T)-Heading;
    U:=G.Update(GP,Dt,P.Observe(E),P.GyroRate,P.MeasurementValid,False);
    Result.RMS:=Result.RMS+Sqr(E);Result.IAE:=Result.IAE+Abs(E)*Dt;
    if T>=Duration-2 then begin Tail:=Tail+Sqr(E);Inc(TailCount);end;
    if Abs(E)>0.5 then Result.Settle:=T+Dt;
    if Abs(U)>=GP.MaxRate-0.001 then Inc(SatCount);
    Result.PeakError:=Max(Result.PeakError,Abs(E));Result.PeakRate:=Max(Result.PeakRate,Abs(P.Rate));
    TV:=TV+Abs(U-PreviousU);PreviousU:=U;
    if Trace then Writeln(F,Format('%.4f,%.6f,%.6f,%.6f,%.6f,%.6f',[T,Bearing(Mode,T),Heading,E,P.Rate,U]));
    P.Advance(U,Dt);Heading:=Heading+P.Rate*Dt;
  end;
  if Trace then CloseFile(F);
  Result.RMS:=Sqrt(Result.RMS/Count);Result.TailRMS:=Sqrt(Tail/TailCount);
  Result.Sat:=SatCount/Count;
  Result.Cost:=Result.IAE/(Duration*Scale)+1.5*Result.TailRMS/Scale+
    0.35*Result.Sat+0.004*TV/Duration+0.4*Max(0,Result.PeakError/Scale-1);
end;

function Score(const P:TGateParameters):Double;
var A,M,S:Integer;R:TMetrics;Worst,Sum:Double;
begin
  Sum:=0;Worst:=0;
  for A:=1 to 2 do for M:=0 to 3 do for S:=0 to 1 do begin
    R:=Run(P,A,M,S,20261023,6);Sum:=Sum+R.Cost;Worst:=Max(Worst,R.Cost);
  end;
  Result:=Sum/16+0.35*Worst;
end;

function LogUniform(L,H:Double):Double;
begin Result:=Exp(Ln(L)+Random*(Ln(H)-Ln(L)));end;

procedure Save(const P:TGateParameters;const Name:string);
var Ini:TMemIniFile;
begin
  Ini:=TMemIniFile.Create(OutputDir+Name+'.ini');
  try
    Ini.WriteFloat('GateNavigation','N',P.N);Ini.WriteFloat('GateNavigation','LeadTime',P.LeadTime);
    Ini.WriteFloat('GateNavigation','KLambda',P.KLambda);Ini.WriteFloat('GateNavigation','MaxRate',P.MaxRate);
    Ini.WriteFloat('GateNavigation','FilterTime',P.FilterTime);Ini.WriteFloat('GateNavigation','RateKp',P.RateKp);
    Ini.WriteFloat('GateNavigation','RateKi',P.RateKi);Ini.WriteInteger('GateNavigation','SensorDelay',P.SensorDelay);
    Ini.UpdateFile;
  finally Ini.Free;end;
end;

procedure Report(const P:TGateParameters;const Name:string);
var A,M,S,Seed:Integer;R:TMetrics;
begin
  for A:=1 to 2 do for M:=0 to 6 do for S:=0 to 5 do for Seed:=1 to 2 do begin
    R:=Run(P,A,M,S,20262000+Seed,8, (Name='rounded') and (M=1) and (S=0) and (Seed=1));
    Writeln(Detail,Format('%s,%d,%d,%d,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f',
      [Name,A,M,S,Seed,R.Cost,R.RMS,R.TailRMS,R.IAE,R.Sat,R.PeakRate,R.PeakError,R.Settle]));
  end;
end;

var Best,Candidate,Rounded,DefaultP,Gentle:TGateParameters; BestScore,C:Double;I,J:Integer;
begin
  try
    FormatSettings.DecimalSeparator:='.';RandSeed:=20261024;
    if ParamCount<>1 then raise Exception.Create('Pass an output directory');
    OutputDir:=IncludeTrailingPathDelimiter(ExpandFileName(ParamStr(1)));ForceDirectories(OutputDir);
    DefaultP:=DefaultGateParameters;Gentle:=DefaultP;
    Gentle.N:=1;Gentle.LeadTime:=0;Gentle.RateKp:=0.5;Gentle.RateKi:=0;Gentle.MaxRate:=20;Gentle.SensorDelay:=0;
    Best:=Gentle;BestScore:=Score(Best);Writeln('Diagnostic score ',BestScore:0:6);
    for I:=1 to 1800 do begin
      Candidate:=Best;
      if (I<=500) or (I mod 5=0) then begin
        Candidate.N:=Random*3;Candidate.LeadTime:=Random*0.2;
        if I mod 2=0 then Candidate.LeadTime:=0;
        Candidate.KLambda:=LogUniform(0.5,8);Candidate.FilterTime:=LogUniform(0.05,0.5);
        Candidate.RateKp:=LogUniform(0.05,1.4);Candidate.RateKi:=LogUniform(0.1,10);
        Candidate.MaxRate:=20+Random(9)*5;Candidate.SensorDelay:=Random(4);
      end else begin
        J:=Random(8);
        case J of
          0: Candidate.N:=EnsureRange(Candidate.N+(Random-0.5)*0.6,0.0,4.0);
          1: Candidate.LeadTime:=EnsureRange(Candidate.LeadTime+(Random-0.5)*0.06,0.0,0.3);
          2: Candidate.KLambda:=EnsureRange(Candidate.KLambda*Exp((Random-0.5)*0.7),0.3,10.0);
          3: Candidate.FilterTime:=EnsureRange(Candidate.FilterTime*Exp((Random-0.5)*0.6),0.03,0.8);
          4: Candidate.RateKp:=EnsureRange(Candidate.RateKp*Exp((Random-0.5)*0.7),0.02,1.5);
          5: Candidate.RateKi:=EnsureRange(Candidate.RateKi*Exp((Random-0.5)*0.7),0.05,12.0);
          6: Candidate.MaxRate:=EnsureRange(Candidate.MaxRate+(Random(3)-1)*5,20.0,60.0);
          7: Candidate.SensorDelay:=Random(4);
        end;
      end;
      C:=Score(Candidate);
      if C<BestScore then begin Best:=Candidate;BestScore:=C;Save(Best,'best');end;
      if I mod 100=0 then Writeln('Iteration ',I,' best ',BestScore:0:6);
    end;
    Rounded:=Best;
    Rounded.N:=Round(Best.N*10)/10;Rounded.LeadTime:=Round(Best.LeadTime*100)/100;
    Rounded.KLambda:=Round(Best.KLambda*10)/10;Rounded.FilterTime:=Round(Best.FilterTime*100)/100;
    Rounded.RateKp:=Round(Best.RateKp*100)/100;Rounded.RateKi:=Round(Best.RateKi*10)/10;
    Save(Rounded,'rounded');Save(DefaultP,'defaults');Save(Gentle,'diagnostic');
    Writeln('Rounded score ',Score(Rounded):0:6,' best ',BestScore:0:6);
    AssignFile(Detail,OutputDir+'validation.csv');Rewrite(Detail);
    Writeln(Detail,'setting,axis,mode,stress,seed,cost,rms,tail_rms,iae,saturation,peak_rate,peak_error,settle');
    Report(DefaultP,'defaults');Report(Gentle,'diagnostic');Report(Best,'best');Report(Rounded,'rounded');
    CloseFile(Detail);Writeln('DONE');
  except on E:Exception do begin Writeln(E.ClassName,': ',E.Message);ExitCode:=1;end;end;
end.
