# This is the script to run ChangeLane procedure defined in the
#    srunner/scenarios/change_lane.py file
#    srunner/data/scenario_changelane.json
# Files modified
#    srunner/scenarios/route_scenario.py
#    srunner/data/routes_town04_circular.xml

function Get-Order
{
    [string[]]$order = 'Safe, then Unsafe', 'Unsafe, then Safe'

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
    $FilePath = "C:\Users\olequ\Downloads\CARLA 0.9.13\CarlaUE4\Config\DrEyeVr.ini"
    $FilteredLines = Get-Content -Path $FilePath | Select-String -Pattern "RICHA" | Select-String -Pattern "^\s*;"
    return $FilteredLines.Length
}

function CheckCameraSettings
{
    $FilePath = "C:\Users\olequ\AppData\Local\CarlaUE4\Saved\Config\CameraSettings.ini"
    $FilteredLines = Get-Content -Path $FilePath | Select-String -Pattern "RICHA" | Select-object -First 1 | Select-String -Pattern "^\s*;"
    return $FilteredLines.Length
}

if (CheckVechicleSettings -ne 0)
{
    Write-Host "==== MISCONFIGURED ===="
    Write-Host "File 'DrEyeVr.ini' has some lines with 'RICHA' mark commented out"
    Write-Host ""
    Write-Host "Exiting...."
    return
}

$isOK = CheckCameraSettings -ne 0
if (-not $isOK)
{
    Write-Host "==== MISCONFIGURED ===="
    Write-Host "File 'CameraSettingd.ini' is not the one configured for 'RICHA'"
    Write-Host ""
    Write-Host "Exiting...."
    return
}

return

Set-Location ..

. ".set-env.ps1"
. ".enter-venv.ps1"

Set-Location .\scenario_runner

$order = Get-Order
if ($order -gt 0)
{
    python run_experiment.py `
        --title changelane `
        --route `
            srunner/data/routes_town04_circular.xml `
            srunner/data/scenario_changelane.json `
            0 `
        --reloadWorld `
        --waitForEgo `
        --params=$order
    
    Write-Host ""
    Write-Host "Done"
}

Set-Location ..\scenarios
