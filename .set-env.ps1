$carla = 'C:\Users\olequ\Downloads\CARLA 0.9.13'
$srunner = 'C:\Users\olequ\Downloads\CARLA 0.9.13\experiments\scenario_runner'

if ($env:computername -eq 'DESKTOP-964O5QQ') {
	$carla = 'D:\CarlaGit\carla\Build\UE4Carla\0.9.13-41-gb7c7a23c7-dirty\WindowsNoEditor'
	$srunner = 'D:\CarlaGit\experiments\scenario_runner'
}

if (-Not (Test-Path -Path $carla)) {
    "CARLA's root folder specified in `.set.ps1` is not valid."
    exit
}

if (-Not (Test-Path -Path $srunner)) {
    "ScenarioRunner's root folder specified in `.set-env.ps1` is not valid."
    exit
}

$env:CARLA_ROOT = $carla
$env:SCENARIO_RUNNER_ROOT = $srunner
$env:PYTHONPATH = ";$env:CARLA_ROOT\PythonAPI\examples;$env:CARLA_ROOT\PythonAPI\util;$env:CARLA_ROOT\PythonAPI\carla;$env:CARLA_ROOT\PythonAPI\carla\dist\carla-0.9.13-py3.7-win-amd64.egg"
