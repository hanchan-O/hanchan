$ErrorActionPreference = 'Stop'
$gbk = [System.Text.Encoding]::GetEncoding(936)
$utf8 = New-Object System.Text.UTF8Encoding $false
$p = Join-Path (Split-Path -Parent $PSScriptRoot) 'user\module\motor\motor.h'
$bytes = [System.IO.File]::ReadAllBytes($p)
$text = $utf8.GetString($bytes)
[System.IO.File]::WriteAllText($p, $text, $gbk)
Write-Host "GBK OK: $p"
