$p = if ($args[0]) { $args[0] } else { Join-Path (Split-Path -Parent $PSScriptRoot) 'user\module\motor\motor.h' }
$b = [System.IO.File]::ReadAllBytes($p)
Write-Host "len=$($b.Length)"
Write-Host ("first24=" + (($b[0..[Math]::Min(23,$b.Length-1)] | ForEach-Object { '{0:X2}' -f $_ }) -join ' '))
