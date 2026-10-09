program TargetTest;
{$APPTYPE CONSOLE}
uses
  System.SysUtils, System.Classes, System.IniFiles, Vcl.Forms, Vcl.Controls,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas',
  uGuidanceController in '..\uGuidanceController.pas';
// Завершает проверку с ошибкой, если условие не выполнено.
procedure Check(B: Boolean; const S: string);
begin
  if not B then raise Exception.Create(S);
end;
var
  M: TMotionModel;
  A: Double;
  F: TForm2;
  Ini: TMemIniFile;
  Name: string;
begin
  try
    M.Init;
    Check(M.TryAngleToPoint(100, 100, A) and (Abs(A - 45) < 1E-9), '45 degrees');
    Check(M.TryAngleToPoint(0, -100, A) and (Abs(A + 90) < 1E-9), '-90 degrees');
    M.Fi := Pi / 2;
    Check(M.TryAngleToPoint(100, 0, A) and (Abs(A + 90) < 1E-9), 'Heading');
    Check(not M.TryAngleToPoint(0, 0, A), 'Coincident points');
    M.V := 0;
    Check(not M.TryAngleToPoint(100, 100, A), 'Zero speed');
    Application.Initialize;
    Name := ChangeFileExt(ParamStr(0), '.ini');
    DeleteFile(Name);
    Ini := TMemIniFile.Create(Name);
    try
      Ini.WriteBool('Controller', 'DelayCorrection', True);
      Ini.WriteBool('Sensor', 'HeadingOffsetEnabled', False);
      Ini.WriteString('View', 'PixelsPerMetre', '1');
      Ini.WriteInteger('View', 'OriginX', 200);
      Ini.WriteInteger('View', 'OriginY', 200);
      Ini.WriteString('Model', 'Diameter', '2.5');
      Ini.WriteString('Target', 'Diameter', '4');
      Ini.UpdateFile;
    finally Ini.Free; end;
    F := TForm2.Create(nil);
    try
      Check((F.TargetX = 100) and (F.TargetY = 100), 'Defaults');
      Check(F.Model.HeadingOffsetDegrees = DefaultHeadingOffsetDegrees, 'Default heading offset');
      F.OffsetPlusButton.Click;
      Check(F.Model.HeadingOffsetDegrees = DefaultHeadingOffsetDegrees + 1, 'Plus heading offset');
      F.OffsetMinusButton.Click;
      Check(F.Model.HeadingOffsetDegrees = DefaultHeadingOffsetDegrees, 'Minus heading offset');
      F.AngleCorrectionCheckBox.Checked := False;
      Check(not F.Controller.AngleCorrectionEnabled, 'Angle correction toggle');
      Check((F.Model.Diameter = 2.5) and (F.Model.Target.Diameter = 4), 'Diameter load');
      F.Image1MouseDown(F.Image1, mbLeft, [ssLeft], 302, 102);
      F.Image1MouseMove(F.Image1, [ssLeft], 312, 122);
      F.Image1MouseUp(F.Image1, mbLeft, [], 312, 122);
      Check((F.TargetX = 110) and (F.TargetY = 80), 'Drag in world units');
      F.InitButtonClick(nil);
      Check(not F.Controller.AngleCorrectionEnabled, 'Init changed toggles');
      Check((F.Model.Diameter = 2.5) and (F.Model.Target.Diameter = 4), 'Init changed diameter');
      F.Integrate(0.01);
      Check((F.TargetX = 110) and (F.TargetY = 80), 'Static target');
    finally F.Free; end;
    F := TForm2.Create(nil);
    try
      Check((F.TargetX = 110) and (F.TargetY = 80), 'INI restore');
      Check(not F.AngleCorrectionCheckBox.Checked,
        'Toggle INI restore');
      Check(not F.Controller.AngleCorrectionEnabled,
        'Controller toggle INI restore');
      Check((F.Model.Diameter = 2.5) and (F.Model.Target.Diameter = 4), 'Diameter restore');
      F.Image1MouseDown(F.Image1, mbLeft, [ssLeft], 200, 200);
      F.Image1MouseUp(F.Image1, mbLeft, [], 220, 220);
      Check((F.TargetX = 110) and (F.TargetY = 80), 'Origin drag changed target');
    finally F.Free; end;
    Ini := TMemIniFile.Create(Name);
    try
      Check(not Ini.ValueExists('Controller', 'DelayCorrection'), 'Obsolete delay setting');
      Check(not Ini.ValueExists('Sensor', 'HeadingOffsetEnabled'), 'Obsolete sensor setting');
    finally Ini.Free; end;
    DeleteFile(Name);
    Writeln('PASS: angle, defaults, dragging, static coordinates, INI restore');
  except
    on E: Exception do begin Writeln(E.Message); ExitCode := 1; end;
  end;
end.
