[CmdletBinding()]
param(
    [string]$Baseline = '1810a55'
)

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $repoRoot

function Invoke-Checked {
    param([string]$Name, [scriptblock]$Command)
    Write-Host "[RUN] $Name"
    & $Command
    if ($LASTEXITCODE -ne 0) {
        throw "$Name failed with exit code $LASTEXITCODE"
    }
    Write-Host "[PASS] $Name"
}

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if (-not $Condition) { throw $Message }
}

Write-Host "Flying-car tracked validation suite"
Write-Host "repository: $repoRoot"
Write-Host "baseline: $Baseline"

# Task 2: generated-message contract and sole parameter metadata source.
$statusMessage = Get-Content 'msg/FlyingCarStatus.msg' -Raw
foreach ($constant in @(
    'MODE_FLIGHT = 0', 'MODE_TRANSITION_TO_GROUND = 1', 'MODE_GROUND = 2',
    'MODE_TRANSITION_TO_FLIGHT = 3', 'MODE_FAULT = 4', 'REJECTION_NONE = 0',
    'REJECTION_NOT_FLYING_CAR = 1', 'REJECTION_ARMED = 2', 'REJECTION_NOT_LANDED = 3',
    'REJECTION_MOVING = 4', 'REJECTION_CHAIN_UNHEALTHY = 5', 'REJECTION_TIMEOUT = 6'
)) {
    Assert-True $statusMessage.Contains($constant) "Missing FlyingCarStatus constant: $constant"
}
Assert-True (Get-Content 'msg/CMakeLists.txt' -Raw).Contains('FlyingCarStatus.msg') 'FlyingCarStatus.msg is not registered'
Assert-True (-not (Test-Path 'src/modules/flying_car/flying_car_params.c')) 'Duplicate C parameter source exists'
$moduleYaml = Get-Content 'src/modules/flying_car/module.yaml' -Raw
foreach ($contract in @(
    'SYS_FC_TYPE:[\s\S]*?default: 0', 'FC_MODE_CH:[\s\S]*?default: 0',
    'FC_BOOT_MODE:[\s\S]*?default: 0', 'FC_SW_VEL_MAX:[\s\S]*?default: 0.2',
    'FC_SW_DELAY:[\s\S]*?default: 0.5', 'FC_WHEEL_TRACK:[\s\S]*?default: 0.5',
    'FC_WHEEL_SPD_MAX:[\s\S]*?default: 2.0', 'FC_WHEEL_THR_MAX:[\s\S]*?default: 0.5',
    'FC_WHEEL_REV:[\s\S]*?default: 0'
)) {
    Assert-True ($moduleYaml -match $contract) "Parameter metadata mismatch: $contract"
}
Write-Host '[PASS] Task 2 message and parameter structure'

Invoke-Checked 'Task 6 ROMFS tests' { & python -m unittest discover -s test/romfs -p 'test_flying_car_airframe.py' -v }
Invoke-Checked 'Task 7 Commander gate-order tests' { & python -m unittest discover -s test -p 'test_flying_car_commander_arm_gate.py' -v }
Invoke-Checked 'Task 8 model tests' { & python Tools/simulation/gz_custom_models/flying_car/model_test.py }

$bash = Get-Command bash -ErrorAction SilentlyContinue
if ($null -eq $bash) { throw 'bash is required for tracked ROMFS syntax validation but was not found on PATH' }
foreach ($script in @(
    'ROMFS/px4fmu_common/init.d/airframes/80003_flying_car',
    'ROMFS/px4fmu_common/init.d-posix/airframes/80003_gz_flying_car',
    'ROMFS/px4fmu_common/init.d/rc.flying_car_apps',
    'ROMFS/px4fmu_common/init.d/rc.flying_car_defaults'
)) {
    Invoke-Checked "bash -n $script" { & $bash.Source -n $script }
}

$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ('px4-flying-car-validation-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $tempRoot | Out-Null
$metadataXml = Join-Path $tempRoot 'airframes.xml'
Invoke-Checked 'airframe metadata generation' {
    & python Tools/px_process_airframes.py -a ROMFS/px4fmu_common/init.d/airframes -x $metadataXml
}
[xml]$metadata = Get-Content $metadataXml -Raw
Assert-True ($metadata.OuterXml -match '80003') 'Generated airframe metadata does not contain 80003'
Write-Host "[PASS] airframe metadata contains 80003 (artifact retained at $metadataXml)"

[xml](Get-Content 'Tools/simulation/gz_custom_models/flying_car/model.config' -Raw) | Out-Null
[xml](Get-Content 'Tools/simulation/gz_custom_models/flying_car/model.sdf' -Raw) | Out-Null
Write-Host '[PASS] Task 8 model.config and model.sdf XML parsing'

$gpp = Get-Command g++.exe -ErrorAction SilentlyContinue
if ($null -eq $gpp) { $gpp = Get-Command g++ -ErrorAction SilentlyContinue }
if ($null -eq $gpp) { throw 'g++ is required for the tracked Commander helper test but was not found on PATH' }
$commanderTest = Join-Path $tempRoot 'flying-car-commander-safety-test.exe'
Invoke-Checked 'Task 7 Commander helper strict C++ test' {
    & $gpp.Source -std=c++17 -Wall -Wextra -Werror -pedantic -I. test/flying_car_commander_safety_test.cpp -o $commanderTest
    if ($LASTEXITCODE -eq 0) { & $commanderTest }
}

Invoke-Checked 'baseline revision exists' { & git cat-file -e "$Baseline^{commit}" }
$genericDiff = & git diff --name-only "$Baseline..HEAD" -- src/modules/mc_pos_control src/modules/mc_att_control src/modules/mc_rate_control src/modules/rover_differential src/modules/control_allocator
Assert-True ($LASTEXITCODE -eq 0) 'Generic-controller diff command failed'
Assert-True ([string]::IsNullOrWhiteSpace(($genericDiff -join "`n"))) "Generic controllers changed:`n$($genericDiff -join "`n")"
Write-Host '[PASS] generic controllers unchanged from baseline'

$airframeDiff = & git diff --name-only "$Baseline..HEAD" -- ROMFS/px4fmu_common/init.d/airframes/6003_fully_actuated_hexa ROMFS/px4fmu_common/init.d/airframes/6004_fully_actuated_hexa_arm
Assert-True ($LASTEXITCODE -eq 0) '6003/6004 diff command failed'
Assert-True ([string]::IsNullOrWhiteSpace(($airframeDiff -join "`n"))) "6003/6004 changed:`n$($airframeDiff -join "`n")"
Write-Host '[PASS] 6003 and 6004 airframes unchanged from baseline'

$forbiddenPattern = '(commander|mc_pos_control|mc_att_control|mc_rate_control|control_allocator|rover_differential)\s+(start|stop)'
$scanFiles = @(
    Get-ChildItem 'src/modules/flying_car' -Recurse -File
    Get-Item 'ROMFS/px4fmu_common/init.d/rc.flying_car_apps'
)
$forbidden = $scanFiles | Select-String -Pattern $forbiddenPattern
Assert-True ($null -eq $forbidden) "Flying-car code controls a forbidden module:`n$($forbidden -join "`n")"
Write-Host '[PASS] forbidden module-main scan is empty'

Invoke-Checked 'git diff --check' { & git diff --check }
Write-Host '[PASS] tracked flying-car validation suite complete'
