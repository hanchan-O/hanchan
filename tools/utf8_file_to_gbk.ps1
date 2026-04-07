param([Parameter(Mandatory=$true)][string]$RelativePath)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $PSScriptRoot
$p = Join-Path $here $RelativePath
if (-not (Test-Path -LiteralPath $p)) { throw "Missing: $p" }
$utf8 = [System.Text.Encoding]::UTF8
$gbk = [System.Text.Encoding]::GetEncoding(936)
$bytes = [System.IO.File]::ReadAllBytes($p)
$text = $utf8.GetString($bytes)
[System.IO.File]::WriteAllText($p, $text, $gbk)
Write-Host "GBK OK: $p"
