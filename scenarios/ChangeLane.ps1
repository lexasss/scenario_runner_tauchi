# This is the script to run ChangeLane procedure defined in the
#    srunner\scenarios\change_lane.py file
# Files
#    srunner/data/routes_changelane.xml
#    srunner/data/scenario_triggers_changelane.json
#    srunner/scenarios/route_scenario.py
# were modified accordingly

function Get-Task
{
    [string[]]$tasks = 'Just drive', 'Safe lane change', 'Unsafe lane change'

    Write-Host "Available tasks:"
    Write-Host ""

    For ($i=0; $i -lt $tasks.Count; $i++)
    {
        Write-Host "$($i+1): $($tasks[$i])"
    }
    
    Write-Host ""

    [ValidateScript({$_ -ge 0 -and $_ -le $tasks.Length})]
    [int]$number = Read-Host "Press the number to select a task"
    
    Write-Host ""

    if($? -and $number -gt 0) {
        Write-Host "Starting the task '$($tasks[$number - 1])'"
        return $number
    }
    else {
        Write-Host "No task selected. Bye!"
    }

    return 0
}


Set-Location ..

. ".set-env.ps1"
. ".enter-venv.ps1"

Set-Location .\scenario_runner

#$task = Get-Task
$task = 3
if ($task -gt 0)
{
    python run_experiment.py --title changelane --route srunner/data/routes_changelane.xml srunner/data/scenario_triggers_changelane.json 0 --reloadWorld --waitForEgo --params=$task
    
    Write-Host ""
    Write-Host "Done"
}

Set-Location ..\scenarios
