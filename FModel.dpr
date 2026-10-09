program FModel;

uses
  Vcl.Forms,
  uMain in 'uMain.pas' {Form2},
  uMotionModel in 'uMotionModel.pas',
  uGuidanceController in 'uGuidanceController.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  Application.CreateForm(TForm2, Form2);
  Application.Run;
end.
