# This is the script to run ChangeLane procedure defined in the
#    srunner/scenarios/change_lane.py file
#    srunner/data/scenario_changelane.json
# Files modified
#    srunner/scenarios/route_scenario.py
#    srunner/data/routes_town04_circular.xml

# Helpers

$SettingsMark = "RICHA"

function Get-Order
{
    $order = 'Safe, then Unsafe', 'Unsafe, then Safe'

    Write-Host "Available lane change order:"
    Write-Host ""

    For ($i=0; $i -lt $order.Count; $i++)
    {
        Write-Host "$($i+1): $($order[$i])"
    }
    
    Write-Host ""

    [ValidateScript({$_ -ge 0 -and $_ -le $order.Length})]
    [int]$number = Read-Host "Select the order"
    
    Write-Host ""

    if($? -and $number -gt 0) {
        Write-Host "Starting scenario with order '$($order[$number - 1])'"
        return $number
    }
    else {
        Write-Host "No order selected. Bye!"
    }

    return 0
}

function CheckVechicleSettings
{
    $FilePath = "D:\CarlaGit\carla\Build\UE4Carla\0.9.13-50-gba3e0f5b2-dirty\WindowsNoEditor\CarlaUE4\Config\DReyeVRConfig.ini"
    $FilteredLines = Get-Content -Path $FilePath | Select-String -Pattern $SettingsMark | Select-String -Pattern "^\s*;"
    return $FilteredLines.Length -eq 0
}

function CheckCameraSettings
{
    $FilePath = "C:\Users\TAUCHI\AppData\Local\CarlaUE4\Saved\Config\CaptureCameras.ini"
    $FilteredLines = Get-Content -Path $FilePath | Select-String -Pattern $SettingsMark | Select-object -First 1 | Select-String -Pattern "^\s*;"
    return $FilteredLines.Length -ne 0
}


# Routine


$isOK = CheckVechicleSettings
if (-not $isOK)
{
    Write-Host "==== MISCONFIGURED ===="
    Write-Host "File 'DReyeVRConfig.ini' has some lines with '$SettingsMark' mark commented out"
    Write-Host ""
    Write-Host "Exiting...."
    return
}

$isOK = CheckCameraSettings
if (-not $isOK)
{
    Write-Host "==== MISCONFIGURED ===="
    Write-Host "File 'CaptureCameras.ini' is not the one configured for '$SettingsMark'"
    Write-Host ""
    Write-Host "Exiting...."
    return
}

$order = Get-Order
if ($order -le 0)
{
    return
}

$carlaProc = Get-Process -Name "CarlaUE4" -ErrorAction SilentlyContinue
if ($null -eq $carlaProc)
{
    $okToStartCarla = Read-Host "CARLA is not running. Do you want to start it? [y/N]"
    if ($okToStartCarla -eq "y")
    {
        $carlaProc = Start-Process -FilePath "D:\CarlaGit\carla\Build\UE4Carla\0.9.13-50-gba3e0f5b2-dirty\WindowsNoEditor\CarlaUE4.exe" -PassThru
        Start-Sleep -Seconds 7.0
    }
}

$telemetry = Start-Process `
    -FilePath "python" `
    -ArgumentList "D:\CarlaGit\carla\test_udp_telemetry.py" `
    -WorkingDirectory "D:\CarlaGit\experiments\scenarios" `
    -PassThru

Set-Location ..

. ".set-env.ps1"
. ".enter-venv.ps1"
    
Set-Location .\scenario_runner

python run_experiment.py `
    --title changelane `
    --route `
        srunner/data/routes_town04_circular.xml `
        srunner/data/scenario_changelane.json `
        0 `
    --reloadWorld `
    --waitForEgo `
    --sync `
    --params=$order

Set-Location ..\scenarios

Stop-Process -InputObject $telemetry -ErrorAction SilentlyContinue

# if ($null -ne $carlaProc)
# {
#     Stop-Process -InputObject $carlaProc -ErrorAction SilentlyContinue
# }
    
Write-Host ""
Write-Host "Done"
