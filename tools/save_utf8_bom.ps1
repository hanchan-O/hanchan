# 将已有 UTF-8 文本文件重写为 UTF-8 BOM，便于 Keil/记事本识别；参数：文件路径
param(
    [Parameter(Mandatory = $true)]
    [string] $Path
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Path)) { throw "Not found: $Path" }
$utf8NoBom = New-Object System.Text.UTF8Encoding $false
$utf8Bom = New-Object System.Text.UTF8Encoding $true
$text = [System.IO.File]::ReadAllText($Path, $utf8NoBom)
[System.IO.File]::WriteAllText($Path, $text, $utf8Bom)
Write-Host "UTF-8 BOM OK: $Path"
