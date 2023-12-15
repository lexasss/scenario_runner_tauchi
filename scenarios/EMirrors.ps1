# This is the script to run EMirrors procedure defined in the
#    srunner/scenarios/emirrors.py file
#    srunner/data/scenario_emirrors.json
# Files modified
#    srunner/scenarios/route_scenario.py
#    srunner/data/routes_town04_circular.xml

# Helpers

$SettingsMark = "OLEG"

function Get-Condition
{
    [string[]]$condition = "in all conventional locations", "rear-view location only"

    Write-Host "E-Mirrors are located"
    Write-Host ""

    For ($i=0; $i -lt $condition.Count; $i++)
    {
        Write-Host "$($i+1): $($condition[$i])"
    }
    
    Write-Host ""

    [ValidateScript({$_ -ge 0 -and $_ -le $condition.Length})]
    [int]$number = Read-Host "Select the condition"
    
    Write-Host ""

    if($? -and $number -gt 0) {
        Write-Host "Starting scenario with the condition '$($condition[$number - 1])'"
        return $number
    }
    else {
        Write-Host "No condition selected. Bye!"
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

$condition = Get-Condition
if ($condition -le 0)
{
    return
}

$carlaProc = Get-Process -Name "CarlaUE4" -ErrorAction SilentlyContinue
if ($null -eq $carlaProc)
{
    # Write-Host "Starting CARLA. Please wait..."
    $carlaProc = Start-Process -FilePath "D:\CarlaGit\carla\Build\UE4Carla\0.9.13-50-gba3e0f5b2-dirty\WindowsNoEditor\CarlaUE4.exe" -PassThru
    Start-Sleep -Seconds 7.0
    # $okToStartCarla = Read-Host "CARLA is not running. Do you want to start it? [y/N]"
    # if ($okToStartCarla -eq "y")
    # {
    #     $carlaProc = Start-Process -FilePath "D:\CarlaGit\carla\Build\UE4Carla\0.9.13-50-gba3e0f5b2-dirty\WindowsNoEditor\CarlaUE4.exe" -PassThru
    #     Start-Sleep -Seconds 7.0
    # }
}

Set-Location ..

. ".set-env.ps1"
. ".enter-venv.ps1"

Set-Location .\scenario_runner

python run_experiment.py `
    --title emirror `
    --route srunner/data/routes_town04_circular.xml `
            srunner/data/scenario_emirrors.json `
            0 `
    --reloadWorld `
    --waitForEgo `
    --params=$condition

Set-Location ..\scenarios

if ($null -ne $carlaProc)
{
    Write-Host "Shutting down CARLA..."
    Stop-Process -InputObject $carlaProc
}

Write-Host ""
Write-Host "Done"
