program AdaptiveTuningValidation;
{$APPTYPE CONSOLE}
uses System.SysUtils,System.Classes,System.IniFiles,System.IOUtils,Vcl.Forms,uMain in '..\uMain.pas';
var Source,Dest:TMemIniFile; Keys:TStringList; I:Integer; F:TForm2; First,Last:TSimulationFrame;
 Name:string;
begin
 try
  Application.Initialize; Name:=ChangeFileExt(ParamStr(0),'.ini');
  TFile.Copy('Win32\Debug\FModel.ini',Name,True);
  Source:=TMemIniFile.Create('profiles\AdaptiveGlide-tuned.ini'); Dest:=TMemIniFile.Create(Name); Keys:=TStringList.Create;
  try
   Source.ReadSection('AdaptiveGlide',Keys);
   for I:=0 to Keys.Count-1 do Dest.WriteString('AdaptiveGlide',Keys[I],Source.ReadString('AdaptiveGlide',Keys[I],''));
   Dest.WriteBool('Simulation','AutoStopEnabled',True);
   Dest.UpdateFile;
  finally Keys.Free; Dest.Free; Source.Free; end;
  F:=TForm2.Create(nil);
  try
   for I:=1 to 3000 do begin
    F.Integrate(0.01); First:=F.DisplayFrame;
    if First.AdaptiveResult.Finished then Break;
   end;
   if not First.AdaptiveResult.Finished then raise Exception.Create('Adaptive did not finish');
   if First.AdaptiveResult.MinimumDistance>=5 then raise Exception.Create('Miss outside threshold');
   Writeln('Applied profile: t_stop=',First.AdaptiveResult.StopTime:0:3,' min=',First.AdaptiveResult.MinimumDistance:0:6);
   for I:=1 to 300 do F.Integrate(0.01);
   Last:=F.DisplayFrame;
   if not CompareMem(@First.AdaptiveController,@Last.AdaptiveController,SizeOf(First.AdaptiveController)) then raise Exception.Create('Finished state changed');
   Writeln('PASS: actual UI settings loading, approach, independent freeze');
  finally F.Free; end;
 except on E:Exception do begin Writeln(E.Message); ExitCode:=1; end; end;
end.
