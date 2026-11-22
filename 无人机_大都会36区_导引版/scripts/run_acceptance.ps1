param(
    [string]$Python = 'D:\Anaconda3\envs\hxy2\python.exe',
    [string]$OutputRoot = '',
    [int]$Target = 10000,
    [double]$Period = 100,
    [switch]$Resume
)
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$taskRoot = Split-Path -Parent $PSScriptRoot
if (-not $OutputRoot) {
    $OutputRoot = Join-Path $taskRoot ('artifacts\acceptance_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
}
$null = New-Item -ItemType Directory -Path $OutputRoot -Force
$taskStopMarker = Join-Path $OutputRoot 'STOP_FULL_LOAD_QUEUE'
if (Test-Path -LiteralPath $taskStopMarker) {
    Write-Output 'Full-load queue stopped by current user test-scope instruction.'
    exit 0
}
$env:PYTHONIOENCODING = 'utf-8'
$cases = @()
foreach ($mode in @('centralized_full', 'centralized_regional', 'onboard_regional')) {
    foreach ($step in @('1', '0.1')) {
        $caseName = $mode + '_' + $step + 's'
        $caseDir = Join-Path $OutputRoot $caseName
        $log = Join-Path $OutputRoot ($caseName + '.log')
        if (Test-Path -LiteralPath $caseDir) {
            $resultFile=Join-Path $caseDir 'benchmark_result.json'
            if (-not $Resume -or -not (Test-Path -LiteralPath $resultFile)) {
                throw "Case exists without an authorized completed-record resume: $caseDir"
            }
            $saved=Get-Content -LiteralPath $resultFile -Raw | ConvertFrom-Json
            if ($saved.config.num_uavs -ne $Target -or $saved.config.replenishment_period_seconds -ne $Period) {
                throw "Completed case uses different settings: $caseDir"
            }
            $caseExit=if (-not $saved.success) {2} elseif (-not $saved.load_requirement_met) {3} elseif (-not $saved.acceptance_passed) {4} else {0}
            $cases += @{case=$caseName;exit_code=$caseExit;log=$log;output=$caseDir}
            Write-Output "KEEP completed record $caseName exit=$caseExit"
            continue
        }
        Write-Output "RUN $caseName"
        & $Python -B (Join-Path $PSScriptRoot 'benchmark_metropolis.py') `
            --mode $mode --dt $step --target $Target --period $Period `
            --mission-limit $Target --spawn-seconds 4000 --drain-seconds 120000 --stop-after-peak `
            --sample-safety --output $caseDir > $log 2>&1
        $caseExit = $LASTEXITCODE
        $cases += @{ case=$caseName; exit_code=$caseExit; log=$log; output=$caseDir }
        $cases | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $OutputRoot 'cases.json') -Encoding utf8
        Write-Output "END $caseName exit=$caseExit"
    }
}
Write-Output "Acceptance output: $OutputRoot"
if ($cases.Where({ $_.exit_code -ne 0 }).Count -gt 0) { exit 1 }
