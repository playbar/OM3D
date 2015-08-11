echo off
echo * Testing the license system and basic mosek install. This may take some time.
echo ************************************************************************************* > "%USERPROFILE%\moseklog.txt"
echo * LOGFILE BEGINS *  >>  "%USERPROFILE%\moseklog.txt"
echo Test license server availability >>  "%USERPROFILE%\moseklog.txt"
lmutil hostid -flexid >> "%USERPROFILE%\moseklog.txt"
lmutil hostid >> "%USERPROFILE%\moseklog.txt"
lmutil lmstat -a -c "%MOSEKLM_LICENSE_FILE%" >> "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo Test  LP >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
mosek -f -d MSK_IPAR_LICENSE_DEBUG MSK_ON "%MOSEK7_0_INSTALLDIR%\tools\examples\data\25fv47.mps" >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo Test MIP.  >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
mosek -f -d MSK_IPAR_LICENSE_DEBUG MSK_ON "%MOSEK7_0_INSTALLDIR%\tools\examples\data\flugpl.mps" >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo Show contends of license file. >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
type "%MOSEKLM_LICENSE_FILE%" >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo Show system information. >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
moseksi  >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo Windows architecture: >>  "%USERPROFILE%\moseklog.txt"
echo %PROCESSOR_ARCHITECTURE% >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo License Install Log: >>  "%USERPROFILE%\moseklog.txt"
type "%MOSEK7_0_INSTALLDIR%\mosek_license_install.log" >> "%USERPROFILE%\moseklog.txt"

echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo Current lmgrd: >>  "%USERPROFILE%\moseklog.txt"
type "%MOSEK7_0_INSTALLDIR%\lmgrd.log" >> "%USERPROFILE%\moseklog.txt"
echo * LOGFILE ENDS *  >>  "%USERPROFILE%\moseklog.txt"

echo ******************************************************************
echo * MOSEK has now written the file moseklog.txt to 
echo * "%USERPROFILE%\moseklog.txt"         
echo * If you have problems with running MOSEK, please send
echo * this file along with your support request to support@mosek.com. 
echo ******************************************************************
pause                      
