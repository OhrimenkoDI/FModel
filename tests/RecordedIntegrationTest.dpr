program RecordedIntegrationTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Classes, System.Math, System.IniFiles,
  Winapi.Windows, Winapi.Messages, Vcl.Forms, Vcl.Controls, Vcl.ComCtrls, Vcl.Graphics, Vcl.Imaging.pngimage,
  uMain in '..\uMain.pas', uParameters in '..\uParameters.pas';

procedure Check(B: Boolean; const Msg: string);
begin if not B then raise Exception.Create(Msg); end;
var F: TForm2; P: TParametersForm; V,ReadBack: TParameterValues;
  Ini: TMemIniFile; Name: string; I,J,Invalid: Integer;
begin
  try
    Application.Initialize;
    Name:=ChangeFileExt(ParamStr(0),'.ini');
    Check(not FileExists(Name),'Use a clean isolated test output folder');
    V:=DefaultParameterValues;
    V[75]:=2;V[6]:=0;V[34]:=0;V[53]:=0;V[74]:=0;
    V[1]:=100;V[2]:=3;V[7]:=0;V[31]:=20;
    Ini:=TMemIniFile.Create(Name);
    try
      WriteParameterValues(Ini,V);Ini.UpdateFile;
      ReadBack:=DefaultParameterValues;ReadParameterValues(Ini,ReadBack);
      for I:=75 to 86 do Check(V[I]=ReadBack[I],'Recorded INI round trip');
    finally Ini.Free;end;
    F:=TForm2.Create(nil);
    try
      Check(F.Model.RecordedPlant.Parameters.Profile=2,'PID profile');
      Check(F.Gate.RecordedPlant.Parameters.Profile=2,'Gate profile');
      Check(F.GatePlus.RecordedPlant.Parameters.Profile=2,'Gate+ profile');
      for I:=1 to 40 do F.Integrate(0.01);
      Check(F.DisplayFrame.AdaptiveModel.RecordedPlant.Parameters.Profile=2,'Adaptive profile');
      Check(Abs(F.Model.GyroDegrees-RadToDeg(F.Model.W))<1E-10,'PID physical rate');
      Check(F.HistoryCount>1,'Recorded history missing');
      F.InitButtonClick(nil);
      Check(F.Model.RecordedPlant.Rate=0,'Recorded state not reset');
      Check(F.Gate.RecordedPlant.Parameters.Profile=2,'Reset lost recorded settings');
    finally F.Free;end;
    P:=TParametersForm.Create(nil);
    try
      P.SetValues(V);Check(P.TryGetValues(ReadBack,Invalid),'Recorded parameters rejected');
      for I:=75 to 86 do Check(V[I]=ReadBack[I],'UI round trip');
      V[75]:=1.5;P.SetValues(V);
      Check(not P.TryGetValues(ReadBack,Invalid),'Fractional profile accepted');
      Check(Invalid=75,'Wrong invalid field');
      V[75]:=2;P.SetValues(V);
      for J:=0 to P.ComponentCount-1 do
        if P.Components[J] is TPageControl then
          TPageControl(P.Components[J]).ActivePageIndex:=8;
      Check(P.ClientHeight>=590,'Settings page height');
    finally P.Free;end;
    System.SysUtils.DeleteFile(Name);
    Writeln('PASS shared four-model profile, state reset/history, INI/UI round trip, validation');
  except on E:Exception do begin Writeln(E.ClassName+': '+E.Message);ExitCode:=1;end;end;
end.
