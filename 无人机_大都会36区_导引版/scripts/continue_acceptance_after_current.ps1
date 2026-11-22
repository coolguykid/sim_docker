param(
    [int]$WaitForProcessId,
    [string]$Python = 'D:\Anaconda3\envs\hxy2\python.exe',
    [string]$OutputRoot = ''
)
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$env:PYTHONIOENCODING = 'utf-8'
$taskRoot = Split-Path -Parent $PSScriptRoot
if (-not $OutputRoot) { $OutputRoot = Join-Path $taskRoot 'artifacts\acceptance_default100s_20261102' }
if ($WaitForProcessId) {
    $taskCurrent = Get-CimInstance Win32_Process -Filter "ProcessId=$WaitForProcessId"
    if ($taskCurrent) {
        if ($taskCurrent.CommandLine -notlike '*benchmark_metropolis.py*--mode centralized_regional*--dt 1*--output*centralized_regional_1s*') {
            throw 'Current process identity differs from the expected owned regional case'
        }
        Write-Output "WAIT owned regional case PID $WaitForProcessId"
        Wait-Process -Id $WaitForProcessId -ErrorAction SilentlyContinue
    }
}
$taskRegionalResult = Join-Path $OutputRoot 'centralized_regional_1s\benchmark_result.json'
if (-not (Test-Path -LiteralPath $taskRegionalResult)) { throw 'Current case exited without its result; diagnose before continuing' }
$taskSaved = Get-Content -LiteralPath $taskRegionalResult -Raw | ConvertFrom-Json
if (-not $taskSaved.success) { throw 'Current regional case did not complete; diagnose before continuing' }
$taskFinalDir = Join-Path $OutputRoot 'centralized_full_1s_final'
$taskFinalResult = Join-Path $taskFinalDir 'benchmark_result.json'
if (Test-Path -LiteralPath $taskFinalDir) {
    if (-not (Test-Path -LiteralPath $taskFinalResult)) { throw 'Final-source case exists without a completed result; refusing duplicate launch' }
    $taskFinalSaved = Get-Content -LiteralPath $taskFinalResult -Raw | ConvertFrom-Json
    if (-not $taskFinalSaved.success) { throw 'Final-source case has not passed arrival checks' }
    Write-Output 'KEEP completed final-source centralized_full_1s'
} else {
    Write-Output 'RUN final-source centralized_full_1s'
    & $Python -B (Join-Path $PSScriptRoot 'benchmark_metropolis.py') `
        --mode centralized_full --dt 1 --target 10000 --period 100 `
        --mission-limit 10000 --spawn-seconds 4000 --drain-seconds 120000 `
        --stop-after-peak --sample-safety --output $taskFinalDir `
        > (Join-Path $OutputRoot 'centralized_full_1s_final.log') 2>&1
    $taskCaseExit = $LASTEXITCODE
    Write-Output "END final-source centralized_full_1s exit=$taskCaseExit"
    if ($taskCaseExit -notin @(0,3)) { throw 'Final-source run failed beyond the separately recorded peak-load criterion' }
}
Write-Output 'CONTINUE remaining cases serially with completed-record resume'
& (Join-Path $PSScriptRoot 'run_acceptance.ps1') -Python $Python -OutputRoot $OutputRoot -Target 10000 -Period 100 -Resume
exit $LASTEXITCODE
