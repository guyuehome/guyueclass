set MATLAB=D:\matlab2018a

cd .

if "%1"=="" ("D:\matlab2018a\bin\win64\gmake"  -f LPF.mk all) else ("D:\matlab2018a\bin\win64\gmake"  -f LPF.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
