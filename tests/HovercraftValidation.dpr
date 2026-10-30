program HovercraftValidation;
{$APPTYPE CONSOLE}
uses System.SysUtils,System.Classes,System.Math,System.IniFiles,System.IOUtils,
 Vcl.Forms,Vcl.Imaging.pngimage,uMain,uParameters,uRecordedPlant;
procedure Check(B:Boolean;const S:string);begin if not B then raise Exception.Create(S);end;
var Root,IniName,Name:string;L:TStringList;Fields:TArray<string>;P:TRecordedPlant;
 Ini:TMemIniFile;V:TParameterValues;F:TForm2;Frame:TSimulationFrame;PNG:TPngImage;
 I,J,K,H,Speed,Course,N,Num:Integer;U,Y,Err,SE,SY,SYY,R2:Double;Report:TextFile;
begin
 try
 FormatSettings.DecimalSeparator:='.';Root:=IncludeTrailingPathDelimiter(ExpandFileName(ParamStr(1)));
 P:=Default(TRecordedPlant);P.Parameters:=DefaultRecordedPlantParameters;P.Parameters.Profile:=3;P.Parameters.OutputLimit:=40;P.Parameters.NoiseScale:=0;P.Reset;
 for I:=0 to 5 do Check(Abs(P.Advance(100,0.01))<1E-10,'Response precedes actuator delay');
 for I:=0 to 1000 do P.Advance(100,0.01);
 Check(Abs(P.LastCommand-40)<1E-10,'Common output limit');Check(Abs(P.Rate-40*0.4852524163107074)<0.001,'Hovercraft DC gain');
 AssignFile(Report,Root+'delphi_replay.csv');Rewrite(Report);Writeln(Report,'log,split,rmse,r2');
 for Name in TDirectory.GetFiles(Root,'replay_*.csv') do begin
  L:=TStringList.Create;try L.LoadFromFile(Name);Fields:=L[1].Split([',']);
   P.Parameters.OutputLimit:=120;P.Reset(StrToFloat(Fields[2]));N:=0;SE:=0;SY:=0;SYY:=0;
   for I:=1 to L.Count-1 do begin
    Fields:=L[I].Split([',']);U:=StrToFloat(Fields[1]);Y:=StrToFloat(Fields[2]);
    if I>=36 then begin Err:=P.Rate-Y;SE:=SE+Sqr(Err);SY:=SY+Y;SYY:=SYY+Sqr(Y);Inc(N);end;
    P.Advance(U,0.01);
   end;
   R2:=1-SE/Max(1E-9,SYY-Sqr(SY)/N);Num:=StrToInt(Copy(ExtractFileName(Name),8,4));
   Writeln(Report,Format('%d,%d,%.8f,%.8f',[Num,Num mod 2,Sqrt(SE/N),R2]));
  finally L.Free;end;
 end;
 CloseFile(Report);Writeln('PASS Hovercraft delay, DC gain, common output limit; all EDF replays exported');
 Application.Initialize;IniName:=ChangeFileExt(ParamStr(0),'.ini');
 Check(not FileExists(IniName),'Use isolated validation INI');
 AssignFile(Report,Root+'geometry.csv');Rewrite(Report);Writeln(Report,'speed_kmh,heading_deg,course_time,pid_m,gate_m,adaptive_m,plus_m');
 for Speed:=0 to 2 do for H:=0 to 2 do for Course:=0 to 2 do begin
  V:=DefaultParameterValues;Ini:=TMemIniFile.Create(Root+'FModel.ini');try ReadParameterValues(Ini,V);finally Ini.Free;end;
  V[31]:=3.6+Speed*1.8;V[35]:=(H-1)*10;V[85]:=0.2+Course*0.3;
  Check(V[75]=3,'Preset profile not loaded');Check(V[87]=40,'Preset limit not loaded');
  Ini:=TMemIniFile.Create(IniName);try WriteParameterValues(Ini,V);Ini.WriteBool('Simulation','AutoStopEnabled',True);Ini.UpdateFile;finally Ini.Free;end;
  F:=TForm2.Create(nil);try
   for I:=1 to 2000 do begin F.Integrate(0.01);if F.DisplayFrame.StoppedOnDistance then Break;end;
   Frame:=F.DisplayFrame;
   Check(Frame.Model.RecordedPlant.Parameters.Profile=3,'PID profile');Check(Frame.Gate.RecordedPlant.Parameters.Profile=3,'Gate profile');
   Check(Frame.AdaptiveModel.RecordedPlant.Parameters.OutputLimit=40,'Adaptive output limit');Check(Frame.GatePlus.RecordedPlant.Parameters.OutputLimit=40,'Plus output limit');
   Writeln(Report,Format('%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f',[V[31],V[35],V[85],Frame.MinimumDistance,Frame.Gate.MinimumDistance,Frame.AdaptiveMinimumDistance,Frame.GatePlus.MinimumDistance]));
   if (Speed=1) and (H=1) and (Course=1) then begin
    F.FormResize(nil);PNG:=TPngImage.Create;try PNG.Assign(F.Image1.Picture.Bitmap);PNG.SaveToFile(Root+'model_preview.png');finally PNG.Free;end;
   end;
  finally F.Free;end;
 end;
 CloseFile(Report);System.SysUtils.DeleteFile(IniName);Writeln('PASS 27 four-controller geometry scenarios and profile integration');
 except on E:Exception do begin Writeln(E.ClassName,': ',E.Message);ExitCode:=1;end;end;
end.
