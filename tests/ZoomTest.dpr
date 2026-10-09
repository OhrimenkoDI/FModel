program ZoomTest;
{$APPTYPE CONSOLE}
uses
  System.SysUtils, System.Types, System.IniFiles, Vcl.Forms,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas',
  uGuidanceController in '..\uGuidanceController.pas';
var
  F: TForm2;
  Ini: TMemIniFile;
  Name: string;
  Handled: Boolean;
begin
  try
    Application.Initialize;
    Name := ChangeFileExt(ParamStr(0), '.ini');
    Ini := TMemIniFile.Create(Name);
    try
      Ini.WriteString('View', 'PixelsPerMetre', '10');
      Ini.WriteInteger('View', 'OriginX', 200);
      Ini.WriteInteger('View', 'OriginY', 200);
      Ini.UpdateFile;
    finally Ini.Free; end;
    F := TForm2.Create(nil);
    try
      Handled := False;
      F.FormMouseWheel(F, [], 120, F.Image1.ClientToScreen(Point(400, 300)), Handled);
      if not Handled then raise Exception.Create('Wheel not handled');
      Ini := TMemIniFile.Create(Name);
      try
        if Abs(StrToFloat(Ini.ReadString('View', 'PixelsPerMetre', ''),
          TFormatSettings.Invariant) - 11) > 1E-9 then
          raise Exception.Create('Scale not saved');
        if (Ini.ReadInteger('View', 'OriginX', 0) <> 180) or
           (Ini.ReadInteger('View', 'OriginY', 0) <> 190) then
          raise Exception.Create('Cursor anchor moved');
      finally Ini.Free; end;
      F.FormMouseWheel(F, [], -120, F.Image1.ClientToScreen(Point(400, 300)), Handled);
      Ini := TMemIniFile.Create(Name);
      try
        if Abs(StrToFloat(Ini.ReadString('View', 'PixelsPerMetre', ''),
          TFormatSettings.Invariant) - 10) > 1E-9 then
          raise Exception.Create('Zoom out failed');
      finally Ini.Free; end;
      Handled := False;
      F.FormMouseWheel(F, [], 120, F.Image1.ClientToScreen(Point(-1, -1)), Handled);
      if Handled then raise Exception.Create('Wheel outside image handled');
    finally F.Free; end;
    DeleteFile(Name);
    Writeln('PASS: wheel zoom, cursor anchor, INI save, image bounds');
  except
    on E: Exception do begin Writeln(E.Message); ExitCode := 1; end;
  end;
end.
