$root = Split-Path $PSScriptRoot -Parent
$p = Join-Path $root 'Core\Src\main.c'
$utf8 = New-Object System.Text.UTF8Encoding $false
$gbk = [System.Text.Encoding]::GetEncoding(936)
$s = [System.IO.File]::ReadAllText($p, $utf8)
[System.IO.File]::WriteAllBytes($p, $gbk.GetBytes($s))
Write-Host "main.c: UTF-8 -> GBK OK"
