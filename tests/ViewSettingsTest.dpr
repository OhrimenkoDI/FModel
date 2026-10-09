program ViewSettingsTest;

{$APPTYPE CONSOLE}

uses
  System.SysUtils, System.Classes, System.IniFiles, Vcl.Forms, Vcl.Controls, Vcl.Graphics,
  uMain in '..\uMain.pas',
  uMotionModel in '..\uMotionModel.pas',
  uGuidanceController in '..\uGuidanceController.pas';

// Завершает проверку с ошибкой, если условие не выполнено.
procedure Check(Value: Boolean; const MessageText: string);
begin
  if not Value then
    raise Exception.Create(MessageText);
end;

var
  Form: TForm2;
  Ini: TMemIniFile;
  IniName: string;
  Action: TCloseAction;
begin
  try
    Application.Initialize;
    IniName := ChangeFileExt(ParamStr(0), '.ini');
    Ini := TMemIniFile.Create(IniName);
    try
      Ini.WriteString('View', 'PixelsPerMetre', '20');
      Ini.WriteInteger('View', 'OriginX', 120);
      Ini.WriteInteger('View', 'OriginY', 100);
      Ini.UpdateFile;
    finally
      Ini.Free;
    end;
    Form := TForm2.Create(nil);
    try
      Check(Form.Image1.Picture.Bitmap.Canvas.Pixels[130, 100] = clBlue,
        'Loaded scale/origin: circle radius must be 10 pixels');
      // Grabbing near the centre must preserve the mouse offset.
      Form.Image1MouseDown(Form.Image1, mbLeft, [ssLeft], 123, 102);
      Form.Image1MouseMove(Form.Image1, [ssLeft], 203, 162);
      Form.Image1MouseUp(Form.Image1, mbLeft, [], 203, 162);
    finally
      Form.Free;
    end;
    Form := TForm2.Create(nil);
    try
      Check(Form.Image1.Picture.Bitmap.Canvas.Pixels[210, 160] = clBlue,
        'Dragged origin was not restored');
      Form.ClientWidth := 800;
      Form.ClientHeight := 500;
      Form.FormResize(Form);
      Check(Form.Image1.Picture.Bitmap.Canvas.Pixels[210, 160] = clBlue,
        'Resize changed scale or origin');
      // Any background position can pan the coordinate system.
      Form.Image1MouseDown(Form.Image1, mbLeft, [ssLeft], 400, 300);
      Form.Image1MouseUp(Form.Image1, mbLeft, [], 500, 400);
      Check(Form.Image1.Picture.Bitmap.Canvas.Pixels[310, 260] = clBlue,
        'Background drag must shift the coordinate system');
      Form.Image1MouseDown(Form.Image1, mbLeft, [ssLeft], 400, 300);
      Form.Image1MouseMove(Form.Image1, [ssLeft], -100, 1100);
      Form.Image1MouseUp(Form.Image1, mbLeft, [], -100, 1100);
      Form.ClientWidth := 600;
      Form.ClientHeight := 400;
      Form.FormResize(Form);
      Action := caHide;
      Form.FormClose(Form, Action);
    finally
      Form.Free;
    end;
    Ini := TMemIniFile.Create(IniName);
    try
      Check(Ini.ReadString('View', 'PixelsPerMetre', '') = '20', 'Scale not saved');
      Check(Ini.ReadInteger('View', 'OriginX', -1) = -200, 'Left edge blocked pan');
      Check(Ini.ReadInteger('View', 'OriginY', -1) = 1060, 'Bottom edge blocked pan');
      Ini.WriteString('View', 'PixelsPerMetre', 'invalid');
      Ini.WriteInteger('View', 'OriginX', -999);
      Ini.WriteInteger('View', 'OriginY', 999999);
      Ini.UpdateFile;
    finally
      Ini.Free;
    end;
    Form := TForm2.Create(nil);
    try
      Form.FormClose(Form, Action);
      Ini := TMemIniFile.Create(IniName);
      try
        Check(Ini.ReadString('View', 'PixelsPerMetre', '') = '10', 'Invalid scale fallback');
        Check(Ini.ReadInteger('View', 'OriginX', -1) = -999, 'Off-screen X restore');
        Check(Ini.ReadInteger('View', 'OriginY', -1) = 999999,
          'Off-screen Y restore');
      finally
        Ini.Free;
      end;
    finally
      Form.Free;
    end;
    DeleteFile(IniName);
    Writeln('PASS: INI restore/save, drag offset, hit testing, resize and invalid settings');
  except
    on E: Exception do
    begin
      Writeln(E.ClassName + ': ' + E.Message);
      ExitCode := 1;
    end;
  end;
end.
