# =============================================================================
# 【重要】本工程 Cursor 已设置 files.encoding = gbk，保存的 .c/.h 已是 GBK。
# 不要再对本仓库里「已是 GBK」的文件运行本脚本！
# 脚本按「整文件 = UTF-8」去读再写成 GBK；若输入其实是 GBK，会把中文毁掉（变成问号或乱码）。
#
# 仅当：你从别处拿到「无 BOM 的 UTF-8」中文源文件，需要一次性转成 GBK 给 Keil 时，再执行。
# =============================================================================
$ErrorActionPreference = 'Stop'
$gbk = [System.Text.Encoding]::GetEncoding(936)
$utf8 = [System.Text.Encoding]::UTF8
$here = Split-Path -Parent $PSScriptRoot
$mod = Join-Path $here 'user\module'
$coreInc = Join-Path $here 'Core\Inc'
$paths = @(
    (Join-Path $mod 'motor\motor.h'),
    (Join-Path $mod 'motor\motor.c'),
    (Join-Path $mod 'flight_control\flight_control.h'),
    (Join-Path $mod 'flight_control\flight_control.c'),
    (Join-Path $mod 'AS5600\AS5600_PWM.h'),
    (Join-Path $mod 'AS5600\AS5600_PWM.c'),
    (Join-Path $mod 'pid\pid.h'),
    (Join-Path $mod 'pid\pid.c'),
    (Join-Path $mod 'elrs\elrs.h'),
    (Join-Path $mod 'elrs\elrs.c'),
    (Join-Path $coreInc 'motor_hw_config.h'),
    (Join-Path $coreInc 'main.h')
)
foreach ($p in $paths) {
    if (-not (Test-Path -LiteralPath $p)) { throw "Missing: $p" }
    $bytes = [System.IO.File]::ReadAllBytes($p)
    $text = $utf8.GetString($bytes)
    [System.IO.File]::WriteAllText($p, $text, $gbk)
    Write-Host "GBK OK: $p"
}
