param(
    [string]$TemplateDir = "C:/cat/template",
    [string]$FirmwarePath = "",
    [string]$ReportDir = "reports"
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($FirmwarePath)) {
    $FirmwarePath = Join-Path $TemplateDir "firmware.bin"
}

Write-Host "[1/6] Building and staging bridge"
& "$PSScriptRoot/build_cat_comm_template_bridge.ps1" -TemplateDir $TemplateDir

Write-Host "[2/6] Preparing allowlist and firmware"
New-Item -ItemType Directory -Path $TemplateDir -Force | Out-Null
$allowlistPath = Join-Path $TemplateDir "allowlist.json"
if (-not (Test-Path $allowlistPath)) {
    Set-Content -Path $allowlistPath -Value "[]"
}
if (-not (Test-Path $FirmwarePath)) {
    $bytes = New-Object byte[] 1024
    for ($i = 0; $i -lt $bytes.Length; $i++) { $bytes[$i] = [byte]($i % 251) }
    [System.IO.File]::WriteAllBytes($FirmwarePath, $bytes)
}

Write-Host "[3/6] Bridge negotiation validation"
cargo run --features vendor-windows --bin simulator_cli -- --validate-cat-bridge --hw-mode=live --vendor-name=cat_comm --vendor-template-dir="$TemplateDir"

Write-Host "[4/6] Running full production phases (strict + execute flash)"
cargo run --features vendor-windows --bin simulator_cli -- --run-production-phases --hw-mode=live --vendor-name=cat_comm --vendor-template-dir="$TemplateDir" --enable-write --noninteractive-approved --dry-run=false --allowlist="$allowlistPath" --target-sa=00 --firmware="$FirmwarePath" --phase-report-dir="$ReportDir" --execute-flash

Write-Host "[5/6] Locating newest report"
$report = Get-ChildItem -Path $ReportDir -Filter "production_phase_report_*.json" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $report) {
    throw "No production report generated"
}

Write-Host "[6/6] Report summary"
Get-Content $report.FullName -Raw | ConvertFrom-Json | Select-Object generated_at_ms, mode, overall_passed, report_sha256 | Format-List
Write-Host "Report file: $($report.FullName)"
