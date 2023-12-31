# This is the script to run VehiclePassesBy procedure defined in the
#    srunner/scenarios/vehicle_passes_by.py file
#    srunner/data/scenario_vehiclepassesby.json
# Files modified
#    srunner/scenarios/route_scenario.py
#    srunner/data/routes_town04_circular.xml

function Get-Side
{
    [string[]]$sides = 'None', 'Left', 'Right'

    Write-Host "The side another vehicle passes by the ego car:"
    Write-Host ""

    For ($i=0; $i -lt $sides.Count; $i++)
    {
        Write-Host "$($i+1): $($sides[$i])"
    }
    
    Write-Host ""

    [ValidateScript({$_ -ge 0 -and $_ -le $sides.Length})]
    [int]$number = Read-Host "Select the side"
    
    Write-Host ""

    if($? -and $number -gt 0) {
        Write-Host "Starting scenario with the side '$($sides[$number - 1])'"
        return $number
    }
    else {
        Write-Host "No side selected. Bye!"
    }

    return 0
}


Set-Location ..

. ".set-env.ps1"
. ".enter-venv.ps1"

Set-Location .\scenario_runner

$side = Get-Side
if ($side -gt 0)
{
    python run_experiment.py `
        --title vehiclepassesby `
        --route `
            srunner/data/routes_town04_circular.xml `
            srunner/data/scenario_vehiclepassesby.json `
            1 `
        --reloadWorld `
        --waitForEgo `
        --params=$side
    
    Write-Host ""
    Write-Host "Done"
}

Set-Location ..\scenarios
