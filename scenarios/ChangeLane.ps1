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
