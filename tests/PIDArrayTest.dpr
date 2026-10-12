program PIDArrayTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, uPID in '..\uPID.pas';
// Сравнивает результаты независимых последовательностей вызовов.
procedure Near(A,B: Double; const S: string);
begin if Abs(A-B)>1E-9 then raise Exception.Create(S); end;
var Sensors, Separate: array[0..2] of TPIDState;
 I,J: Integer; Saved: TPIDState; Dt,Angle,Speed: Double;
// Передаёт одному датчику воспроизводимое измерение и его собственный шаг времени.
procedure Step(var State: TPIDState; SensorIndex, SampleIndex: Integer);
begin
 Dt:=0.006+SensorIndex*0.002+(SampleIndex mod 3)*0.001;
 Angle:=(SensorIndex+1)*4*Sin(SampleIndex*0.013)+SensorIndex;
 Speed:=SensorIndex+0.5*Cos(SampleIndex*0.031);
 PIDupdate(State,Dt,Angle,Speed);
end;
begin
 try
 for I:=0 to 2 do begin
  PIDinit(Sensors[I]); Sensors[I].Parameters.Kp:=2+I;
  Sensors[I].Parameters.BiasAdaptationStart:=0;
  Separate[I]:=Sensors[I];
 end;
 for J:=0 to 999 do for I:=0 to 2 do Step(Sensors[I],I,J);
 for I:=0 to 2 do for J:=0 to 999 do Step(Separate[I],I,J);
 for I:=0 to 2 do begin
  Near(Sensors[I].W,Separate[I].W,'Cross-sensor output state');
  Near(Sensors[I].Wint,Separate[I].Wint,'Cross-sensor integral');
  Near(Sensors[I].Wdiff,Separate[I].Wdiff,'Cross-sensor derivative');
  Near(Sensors[I].AdaptiveAngleCorrection,Separate[I].AdaptiveAngleCorrection,'Cross-sensor bias');
 end;
 Saved:=Sensors[1]; Sensors[0].AngleCorrectionEnabled:=False; PIDreset(Sensors[0]);
 Near(Sensors[0].Wint,0,'Reset integral'); Near(Sensors[0].Wdiff,0,'Reset derivative');
 Near(Sensors[0].AdaptiveAngleCorrection,0,'Reset adaptive correction');
 Near(Sensors[0].Parameters.Kp,2,'Reset changed gains');
 if Sensors[0].AngleCorrectionEnabled then raise Exception.Create('Reset changed toggle');
 Near(Sensors[1].W,Saved.W,'Reset affected another sensor');
 Near(Sensors[1].AdaptiveAngleCorrection,Saved.AdaptiveAngleCorrection,'Reset affected another correction');
 PIDupdate(Sensors[0],0.01,20,5); Near(Sensors[0].Wdiff,0,'Reset kept derivative history');
 Writeln('PASS: 3 independent sensors, 3000 interleaved calls, per-sensor reset, preserved settings');
 except on E:Exception do begin Writeln(E.Message); ExitCode:=1; end; end;
end.
