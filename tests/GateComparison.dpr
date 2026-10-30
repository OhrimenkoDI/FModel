program GateComparison;
{$APPTYPE CONSOLE}
// Deterministic experiment: every law uses the same actuator, slip plant and sensor.
uses System.SysUtils, System.Math, System.Classes,
  uGateNavigationPlus in '..\uGateNavigationPlus.pas', uPID in '..\uPID.pas';
type
  TScenario = record
    Name: string;
    X, Y, VX, VY, Speed, Heading, Slip, Tau, Noise, Dt: Double;
    Delay: Integer;
    Dropout: Boolean;
  end;
  TRun = record Miss, Time, PeakSlew: Double; Near: Boolean; end;
var Scenarios: array of TScenario; Rows: TStringList; Fmt:TFormatSettings;

function Wrap(A: Double):Double;
begin Result:=RadToDeg(ArcTan2(Sin(DegToRad(A)),Cos(DegToRad(A)))); end;
function Num(A:Double):string;
begin Result:=FloatToStr(A,Fmt); end;
procedure Add(const S:TScenario);
var I:Integer;
begin I:=Length(Scenarios); SetLength(Scenarios,I+1); Scenarios[I]:=S; end;

function Run(const S:TScenario; Law:Integer):TRun;
var P:TGatePlusParameters; C:TGatePlusController; PID:TPIDState;
  Angles:array[0..1000]of Double; Validity:array[0..1000]of Boolean;
  I,Index:Integer; X,Y,Heading,Course,Rate,T,Dt,DX,DY,Distance,PreviousDistance:Double;
  Angle,Control,Gain,RX,RY,Fraction,Denom,Closing,OldCommand,Command:Double;
  NearApproach,Passed,NearClosing,Valid:Boolean;
begin
  P:=DefaultGatePlusParameters; P.SensorDelay:=S.Delay; P.KLambda:=0;
  if Law=1 then P.LeadTime:=0;
  if Law=3 then P.KLambda:=0.5;
  C.Reset; PIDinit(PID); PID.AngleCorrectionEnabled:=False;
  // Existing PID coefficients, without its sensor-bias adaptation.
  X:=0;Y:=0;Heading:=S.Heading;Course:=Wrap(Heading-S.Slip);Rate:=0;T:=0;
  Result:=Default(TRun); Result.Miss:=Hypot(S.X,S.Y);
  PreviousDistance:=Result.Miss; NearApproach:=False; Passed:=False; OldCommand:=0;
  I:=0;
  while T<30 do
  begin
    Dt:=S.Dt; if Dt=0 then Dt:=0.005+(I mod 3)*0.002;
    DX:=S.X+S.VX*T-X; DY:=S.Y+S.VY*T-Y; Distance:=Hypot(DX,DY);
    Closing:=DX*(S.VX-S.Speed/3.6*Cos(DegToRad(Course)))+
      DY*(S.VY-S.Speed/3.6*Sin(DegToRad(Course)));
    if (Distance<50) and (Closing<0) then Result.Near:=True;
    if Result.Near and (Distance>PreviousDistance+1E-9) then Break;
    NearClosing:=(Distance<=P.FreezeDistance) and (Closing<0);
    NearApproach:=NearApproach or NearClosing;
    Passed:=Passed or (NearApproach and (Closing>=0));
    Angle:=Wrap(RadToDeg(ArcTan2(DY,DX))-Heading);
    Index:=I mod GatePlusHistoryCapacity;
    Angles[Index]:=Wrap(Angle+S.Noise*(Sin(I*12.9898)+0.5*Sin(I*7.131))/1.5);
    Validity[Index]:=not (S.Dropout and (((T>=2)and(T<2.4)) or (Distance<5)));
    Index:=(I-S.Delay+GatePlusHistoryCapacity) mod GatePlusHistoryCapacity;
    Valid:=(I>=S.Delay) and Validity[Index];
    if Law=0 then
    begin
      if Valid then Command:=EnsureRange(PIDupdate(PID,Dt,Angles[Index],Rate),-P.MaxRate,P.MaxRate)
      else Command:=OldCommand;
      Control:=Command;
    end
    else
    begin
      Control:=C.Update(P,Dt,Angles[Index],Rate,Valid,NearClosing,Passed);
      Command:=C.CommandRate;
    end;
    Result.PeakSlew:=Max(Result.PeakSlew,Abs(Command-OldCommand)/Dt);
    OldCommand:=Command;
    Gain:=1-Exp(-Dt/P.ActuatorTime);Rate:=Rate+Gain*(Control-Rate);
    Heading:=Wrap(Heading+Rate*Dt);
    if S.Tau=0 then Course:=Heading
    else Course:=Wrap(Course+(1-Exp(-Dt/S.Tau))*Wrap(Heading-Course));
    RX:=(S.VX-S.Speed/3.6*Cos(DegToRad(Course)))*Dt;
    RY:=(S.VY-S.Speed/3.6*Sin(DegToRad(Course)))*Dt;
    Denom:=Sqr(RX)+Sqr(RY);Fraction:=0;
    if Denom>0 then Fraction:=EnsureRange(-(DX*RX+DY*RY)/Denom,0.0,1.0);
    Distance:=Hypot(DX+Fraction*RX,DY+Fraction*RY);
    if Distance<Result.Miss then begin Result.Miss:=Distance;Result.Time:=T+Fraction*Dt;end;
    X:=X+S.Speed/3.6*Cos(DegToRad(Course))*Dt;
    Y:=Y+S.Speed/3.6*Sin(DegToRad(Course))*Dt;
    PreviousDistance:=Hypot(DX,DY);T:=T+Dt;Inc(I);
    if IsNan(X) or IsInfinite(X) or IsNan(Control) or IsInfinite(Control) then
      raise Exception.Create('Non-finite experiment: '+S.Name);
  end;
end;

const Laws:array[0..3]of string=('PID','PN','PN_lead','PN_lead_aim');
var Base,S:TScenario; I,J,Count:Integer; R:TRun; Sum,MaxMiss,ContactRadius:Double;
  NearCount,ContactCount:Integer;
begin
  try
    Fmt:=TFormatSettings.Invariant;ContactRadius:=0.5;
    if ParamCount>=2 then ContactRadius:=StrToFloat(ParamStr(2),Fmt);
    Base:=Default(TScenario);Base.Name:='stationary';Base.X:=100;Base.Y:=100;
    Base.Speed:=150;Base.Tau:=0.4;Base.Delay:=3;Base.Dt:=0.01;Add(Base);
    S:=Base;S.Name:='moving';S.VX:=10;Add(S);
    for I:=0 to 2 do begin S:=Base;S.Name:='speed_'+IntToStr(I);S.Speed:=60+I*90;Add(S);end;
    for I:=0 to 3 do begin S:=Base;S.Name:='tau_'+IntToStr(I);S.Tau:=I*0.4;Add(S);end;
    for I:=-1 to 1 do if I<>0 then begin S:=Base;S.Name:='slip_'+IntToStr(I);S.Slip:=I*20;Add(S);end;
    for I:=0 to 2 do begin S:=Base;S.Name:='delay_'+IntToStr(I);S.Delay:=I*6;Add(S);end;
    for I:=1 to 2 do begin S:=Base;S.Name:='noise_'+IntToStr(I);S.Noise:=I*0.5;Add(S);end;
    S:=Base;S.Name:='variable_dt';S.Dt:=0;Add(S);
    S:=Base;S.Name:='dropout';S.Dropout:=True;Add(S);
    S:=Base;S.Name:='rear_left';S.X:=-200;S.Y:=100;S.VX:=13.68;S.VY:=-2.41;Add(S);
    S:=Base;S.Name:='straight';S.Y:=0;Add(S);
    S:=Base;S.Name:='directly_behind';S.X:=-100;S.Y:=0;Add(S);
    S:=Base;S.Name:='heading_60';S.Heading:=60;Add(S);
    S:=Base;S.Name:='heading_minus_60';S.Heading:=-60;Add(S);
    for I:=0 to 7 do begin
      S:=Base;S.Name:='combined_'+IntToStr(I);S.Slip:=20*((I mod 3)-1);
      S.Tau:=0.2+0.2*(I mod 4);S.Speed:=90+30*I;S.Delay:=I;
      S.Noise:=0.25;S.Dt:=0;S.VX:=10;S.Dropout:=(I mod 2)=0;Add(S);
    end;
    Rows:=TStringList.Create;
    try
      Rows.Add('scenario,law,x_m,y_m,vx_mps,vy_mps,speed_kmh,heading_deg,initial_slip_deg,plant_tau_s,delay,noise_amplitude_deg,dt_s,dropout,min_m,min_time_s,peak_command_slew_deg_s2,approached_within_50m,assumed_contact_radius_m,contact');
      for J:=0 to 3 do begin
        Sum:=0;MaxMiss:=0;NearCount:=0;ContactCount:=0;Count:=Length(Scenarios);
        for I:=0 to High(Scenarios) do begin
          S:=Scenarios[I];R:=Run(S,J);Sum:=Sum+R.Miss;MaxMiss:=Max(MaxMiss,R.Miss);
          if R.Miss<5 then Inc(NearCount); if R.Miss<=ContactRadius then Inc(ContactCount);
          Rows.Add(S.Name+','+Laws[J]+','+Num(S.X)+','+Num(S.Y)+','+Num(S.VX)+','+Num(S.VY)+','+
            Num(S.Speed)+','+Num(S.Heading)+','+Num(S.Slip)+','+Num(S.Tau)+','+IntToStr(S.Delay)+','+
            Num(S.Noise)+','+Num(S.Dt)+','+IntToStr(Ord(S.Dropout))+','+Num(R.Miss)+','+Num(R.Time)+','+
            Num(R.PeakSlew)+','+IntToStr(Ord(R.Near))+','+Num(ContactRadius)+','+IntToStr(Ord(R.Miss<=ContactRadius)));
        end;
        Writeln(Laws[J],': cases=',Count,' miss<5m=',NearCount,' contact=',ContactCount,
          ' mean=',Sum/Count:0:3,' max=',MaxMiss:0:3);
      end;
      if ParamCount>=1 then Rows.SaveToFile(ParamStr(1),TEncoding.UTF8);
    finally Rows.Free;end;
    Writeln('Contact radius is an experiment assumption, not calibrated hull/ball geometry.');
  except on E:Exception do begin Writeln(E.ClassName,': ',E.Message);ExitCode:=1;end;end;
end.