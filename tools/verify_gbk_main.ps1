$gbk = [System.Text.Encoding]::GetEncoding(936)
$p = Join-Path (Split-Path $PSScriptRoot -Parent) 'Core\Src\main.c'
$t = [System.IO.File]::ReadAllText($p, $gbk)
$i = $t.IndexOf('USER CODE BEGIN 2')
Write-Host $t.Substring($i, [Math]::Min(550, $t.Length - $i))
