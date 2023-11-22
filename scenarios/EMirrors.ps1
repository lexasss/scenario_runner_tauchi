# This is the script to run EMirrors procedure defined in the
#    srunner/scenarios/emirrors.py file
#    srunner/data/scenario_emirrors.json
# Files modified
#    srunner/scenarios/route_scenario.py
#    srunner/data/routes_town04_circular.xml

function Get-Condition
{
    [string[]]$condition = 'in all conventional locations', 'rear-view location only'

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


Set-Location ..

. ".set-env.ps1"
. ".enter-venv.ps1"

Set-Location .\scenario_runner

$condition = Get-Condition
if ($condition -gt 0)
{
    python run_experiment.py `
        --title emirror `
        --route srunner/data/routes_town04_circular.xml `
                srunner/data/scenario_emirrors.json `
                0 `
        --reloadWorld `
        --waitForEgo `
        --params=$condition
    
    Write-Host ""
    Write-Host "Done"
}

Set-Location ..\scenarios
