param(
    [string]$TemplateDir = "C:/cat/template",
    [switch]$Release
)

$ErrorActionPreference = "Stop"

$mode = if ($Release) { "--release" } else { "" }

Write-Host "[1/3] Building cat_comm_bridge (vendor-windows feature)"
if ($mode -eq "--release") {
    cargo build --features vendor-windows --bin cat_comm_bridge --release
    $src = Join-Path (Get-Location) "target/release/cat_comm_bridge.exe"
} else {
    cargo build --features vendor-windows --bin cat_comm_bridge
    $src = Join-Path (Get-Location) "target/debug/cat_comm_bridge.exe"
}

if (-not (Test-Path $src)) {
    throw "Bridge executable not found at $src"
}

Write-Host "[2/3] Preparing template dir $TemplateDir"
New-Item -ItemType Directory -Path $TemplateDir -Force | Out-Null
$dst = Join-Path $TemplateDir "cat_comm_bridge.exe"

Write-Host "[3/3] Copying executable"
Copy-Item -Path $src -Destination $dst -Force

Write-Host "DONE: $dst"
