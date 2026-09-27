$ErrorActionPreference = 'Stop'

$moduleRoot = Split-Path -Parent $PSScriptRoot
$header = Get-Content -Raw (Join-Path $moduleRoot 'Gimbal.hpp')
$lqr = Get-Content -Raw (Join-Path $moduleRoot 'YawLqrEso.hpp')
$smc = Get-Content -Raw (Join-Path $moduleRoot 'YawSmc.hpp')

if ($header -notmatch 'struct GimbalParam') {
  throw 'missing GimbalParam'
}
if ($header -notmatch 'const GimbalParam PARAM;') {
  throw 'missing const GimbalParam PARAM'
}
foreach ($legacy in @('pit_max_angle_', 'pit_min_angle_', 'pit_lc_',
                       'pit_theta_', 'yaw_k_', 'j_pit_', 'j_yaw_')) {
  if ($header.Contains($legacy)) {
    throw "legacy mechanical member remains: $legacy"
  }
}
if ($smc -match 'float j_kg_m2\{\};') {
  throw 'SMC config still owns Yaw inertia'
}
if ($lqr -match 'float b_nms_rad\{\};') {
  throw 'LQR/ESO config still owns Yaw damping'
}
if ($header -notmatch 'PARAM\.j_yaw' -or $header -notmatch 'PARAM\.yaw_k') {
  throw 'controllers do not consume Gimbal PARAM mechanics'
}

Write-Output 'PASS: Gimbal mechanical parameters have one owner'
