program GateGeometryCheck;
{$APPTYPE CONSOLE}
uses System.SysUtils,System.Math,System.IniFiles,
  uRecordedPlant in '..\uRecordedPlant.pas',uGateNavigation in '..\uGateNavigation.pas';
var G:TGateSimulation;P:TGateParameters;Ini:TMemIniFile;F:TextFile;
    A,M,V,S,K,Setting,Sat,Count:Integer;Y,OldX,OldY,Offset,Time,CourseTime:Double;
    Names:array[0..1] of string;Crossed:Boolean;
begin
  try
    FormatSettings.DecimalSeparator:='.';
    Names[0]:='defaults';Names[1]:='rounded';
    AssignFile(F,IncludeTrailingPathDelimiter(ParamStr(1))+'geometry.csv');Rewrite(F);
    Writeln(F,'setting,axis,initial_angle,speed_kmh,course_time,crossed,time,crossing_error,min_distance,saturation');
    for Setting:=0 to 1 do begin
      P:=DefaultGateParameters;
      if Setting=1 then begin
        Ini:=TMemIniFile.Create(IncludeTrailingPathDelimiter(ParamStr(1))+'rounded.ini');
        try
          P.N:=Ini.ReadFloat('GateNavigation','N',P.N);P.LeadTime:=Ini.ReadFloat('GateNavigation','LeadTime',P.LeadTime);
          P.KLambda:=Ini.ReadFloat('GateNavigation','KLambda',P.KLambda);P.MaxRate:=Ini.ReadFloat('GateNavigation','MaxRate',P.MaxRate);
          P.FilterTime:=Ini.ReadFloat('GateNavigation','FilterTime',P.FilterTime);P.RateKp:=Ini.ReadFloat('GateNavigation','RateKp',P.RateKp);
          P.RateKi:=Ini.ReadFloat('GateNavigation','RateKi',P.RateKi);P.SensorDelay:=Ini.ReadInteger('GateNavigation','SensorDelay',P.SensorDelay);
        finally Ini.Free;end;
      end;
      P.FreezeDistance:=0;P.InitialHeadingDegrees:=0;
      for A:=1 to 2 do for M:=1 to 3 do for V:=0 to 1 do for S:=0 to 2 do begin
        P.SpeedKmh:=20+V*30;Y:=100*Tan(DegToRad(M*5));CourseTime:=0.2+S*0.3;
        G:=Default(TGateSimulation);G.Parameters:=P;G.Reset(100,Y);
        G.RecordedPlant.Parameters.Profile:=A;G.RecordedPlant.Parameters.CourseTime:=CourseTime;
        Sat:=0;Count:=0;Crossed:=False;Time:=0;Offset:=1E6;
        for K:=1 to 4000 do begin
          OldX:=G.X;OldY:=G.Y;G.Step(0.01,100,Y,0,0);Inc(Count);
          if Abs(G.Controller.Output)>=P.MaxRate-0.001 then Inc(Sat);
          if (OldX<100) and (G.X>=100) then begin
            Crossed:=True;Offset:=OldY+(G.Y-OldY)*(100-OldX)/(G.X-OldX)-Y;
            Time:=(K-1+(100-OldX)/(G.X-OldX))*0.01;Break;
          end;
        end;
        Writeln(F,Format('%s,%d,%d,%d,%.3f,%d,%.6f,%.6f,%.6f,%.6f',
          [Names[Setting],A,M*5,20+V*30,CourseTime,Ord(Crossed),Time,Offset,G.MinimumDistance,Sat/Count]));
      end;
    end;
    CloseFile(F);Writeln('PASS 72 geometry scenarios completed');
  except on E:Exception do begin Writeln(E.Message);ExitCode:=1;end;end;
end.
