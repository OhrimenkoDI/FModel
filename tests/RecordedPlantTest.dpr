program RecordedPlantTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, System.Classes,
  uRecordedPlant in '..\uRecordedPlant.pas',
  uGateNavigation in '..\uGateNavigation.pas';

procedure Check(B: Boolean; const Msg: string);
begin if not B then raise Exception.Create(Msg); end;
procedure Near(A,B,Tol: Double; const Msg: string);
begin Check(Abs(A-B)<=Tol, Msg); end;

procedure Lifecycle;
var A,B: TRecordedPlant; I, Fresh: Integer; X,Y: Double;
begin
  A := Default(TRecordedPlant);
  A.Parameters := DefaultRecordedPlantParameters;
  A.Parameters.Profile := 1; A.Reset;
  B := A; Fresh := 0;
  for I:=0 to 299 do
  begin
    X:=A.Observe(5); Y:=B.Observe(5);
    Near(X,Y,1E-12,'Independent deterministic observation');
    if A.FreshMeasurement then Inc(Fresh);
    Near(A.Observe(5),X,1E-12,'Duplicate observation consumed noise');
    Near(A.Advance(20,0),A.Rate,0,'Zero dt changed state');
    X:=A.Advance(20,0.01);Y:=B.Advance(20,0.01);
    Near(X,Y,1E-12,'Independent deterministic dynamics');
    if I<7 then Near(X,0,1E-12,'Command responded before 70 ms delay');
  end;
  Check((Fresh>=179) and (Fresh<=181),'Camera cadence is not 60 Hz');
  Near(A.Rate,A.SteadyRate(20),0.001,'DC response');
  Near(A.SteadyRate(-90),-A.SteadyRate(90),1E-12,'RC curve sign');
  Near(A.SteadyRate(999),A.SteadyRate(120),1E-12,'Firmware RC clamp');
  A.Reset; B.Reset; Near(A.Observe(5),B.Observe(5),0,'Reset must restore seed');
  A.Parameters.NoiseScale:=0; A.Parameters.CameraDelay:=0;
  A.Parameters.CompensationTime:=0.035; A.Reset(10);
  Near(A.Observe(5),4.65,1E-10,'Camera gyro compensation applied once');
  Writeln('PASS lifecycle, physical delay, cadence, sign, saturation, deterministic noise');
end;

procedure Replay(const Name: string; Profile: Integer);
var L: TStringList; Fields: TArray<string>; P: TRecordedPlant;
  I,N: Integer; Command, Actual, Prediction, Error, Train, Test,
  SX,SY,SXX,SYY,SXY,Corr: Double;
begin
  L:=TStringList.Create;
  try
    L.LoadFromFile(Name);
    P:=Default(TRecordedPlant); P.Parameters:=DefaultRecordedPlantParameters;
    P.Parameters.Profile:=Profile;
    Fields:=L[1].Split([',']); P.Reset(StrToFloat(Fields[2],TFormatSettings.Invariant));
    Train:=0;Test:=0;SX:=0;SY:=0;SXX:=0;SYY:=0;SXY:=0;N:=0;
    for I:=1 to L.Count-1 do
    begin
      Fields:=L[I].Split([',']);
      Command:=StrToFloat(Fields[1],TFormatSettings.Invariant);
      Actual:=StrToFloat(Fields[2],TFormatSettings.Invariant);
      Prediction:=P.Rate; Error:=Prediction-Actual;
      Near(Prediction,StrToFloat(Fields[3],TFormatSettings.Invariant),0.001,'Delphi/Python replay');
      if (I>=9) and (I<113) then Train:=Train+Sqr(Error);
      if I>=113 then begin
        Test:=Test+Sqr(Error);Inc(N);SX:=SX+Prediction;SY:=SY+Actual;
        SXX:=SXX+Sqr(Prediction);SYY:=SYY+Sqr(Actual);SXY:=SXY+Prediction*Actual;
      end;
      P.Advance(Command,0.01);
    end;
    Corr:=(SXY-SX*SY/N)/Sqrt((SXX-SX*SX/N)*(SYY-SY*SY/N));
    Writeln(Format('Profile %d: train RMSE %.3f; HOLDOUT RMSE %.3f deg/s; r=%.5f',
      [Profile,Sqrt(Train/104),Sqrt(Test/N),Corr]));
    Check(Corr>0.85,'Held-out waveform agreement regressed');
    Check(Sqrt(Test/N)<85,'Held-out amplitude error regressed');
  finally L.Free; end;
end;

procedure ClosedLoop(Profile: Integer; Gentle: Boolean);
var P: TRecordedPlant; G: TGateController; GP: TGateParameters;
  I, Sat: Integer; Heading, Angle, Command, Peak: Double;
begin
  P:=Default(TRecordedPlant);P.Parameters:=DefaultRecordedPlantParameters;
  P.Parameters.Profile:=Profile;P.Reset;
  G.Reset;GP:=DefaultGateParameters;Heading:=0;Peak:=0;Sat:=0;
  if Gentle then begin GP.N:=1;GP.LeadTime:=0;GP.RateKp:=0.5;
    GP.RateKi:=0;GP.MaxRate:=20;GP.SensorDelay:=0; end;
  for I:=0 to 299 do
  begin
    Angle:=P.Observe(1-Heading);
    Command:=G.Update(GP,0.01,Angle,P.GyroRate,P.MeasurementValid,False);
    if Abs(Command)>=GP.MaxRate-0.01 then Inc(Sat);
    P.Advance(Command,0.01);Heading:=Heading+P.Rate*0.01;
    Peak:=Max(Peak,Abs(P.Rate));
    Check(not IsNan(Heading) and not IsInfinite(Heading),'Closed loop became non-finite');
  end;
  Writeln(Format('Profile %d gentle=%s: fixed-bearing diagnostic, saturation %.1f%%, peak %.2f deg/s',
    [Profile,BoolToStr(Gentle,True),100*Sat/300,Peak]));
end;

begin
  try
    Lifecycle;
    if ParamCount<>1 then raise Exception.Create('Pass fixture directory');
    Replay(IncludeTrailingPathDelimiter(ParamStr(1))+'Roll_replay.csv',1);
    Replay(IncludeTrailingPathDelimiter(ParamStr(1))+'Pitch_replay.csv',2);
    ClosedLoop(1,False);ClosedLoop(1,True);ClosedLoop(2,False);ClosedLoop(2,True);
  except on E:Exception do begin Writeln(E.ClassName+': '+E.Message);ExitCode:=1;end;end;
end.
