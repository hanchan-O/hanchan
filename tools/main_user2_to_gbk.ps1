# Fix USER CODE BEGIN/END 2 block and save main.c as GBK for Keil (GB2312 compatible)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
Set-Location $root
$path = Join-Path $root 'Core\Src\main.c'

$startMarker = "  /* USER CODE BEGIN 2 */"
$endMarker = "  /* USER CODE END 2 */"

$gbk = [System.Text.Encoding]::GetEncoding(936)
$utf8 = New-Object System.Text.UTF8Encoding $false
$raw = [System.IO.File]::ReadAllBytes($path)
$contentUtf8 = $utf8.GetString($raw)
$contentGbk = $gbk.GetString($raw)
# 优先 UTF-8；找不到标记则按 GBK 读（文件已用 Keil 保存过的情况）
$content = $contentUtf8
if ($content.IndexOf($startMarker) -lt 0) { $content = $contentGbk }
$i1 = $content.IndexOf($startMarker)
$i2 = $content.IndexOf($endMarker)
if ($i1 -lt 0 -or $i2 -le $i1) { throw "USER CODE BEGIN/END 2 markers not found" }

$before = $content.Substring(0, $i1 + $startMarker.Length)
$after = $content.Substring($i2)

# 仅使用 GB2312/GBK 常见字符，避免全角波浪号等导致乱码
$middle = @"

	/* TIM2: 四路 PWM，电机 M1/M2（通道 CH1 至 CH4） */
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	/* TIM3: 四路 PWM，电机 M3/M4（通道 CH1 至 CH4） */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	/* TIM14: 定时基准 + 周期中断（原 TIM1 时基已迁至 TIM14，见 tim.c） */
	HAL_TIM_Base_Start_IT(&htim14);

	/* TIM1: 四通道输入捕获，四路 MT6701 编码器 PWM 脉宽测量 */
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

	/* 上电后延时，等待电源与外设稳定 */
	HAL_Delay(1000);
	/* 再次初始化 USART1（PB6/PB7），供 ELRS 接收机串口 */
	MX_USART1_UART_Init();
	ELRS_Init();			/* ELRS 接收机协议与状态初始化 */
	Chassis_PID_Init();		/* 底盘电机 PID 参数与状态初始化 */
"@

$full = $before + $middle + $after
$outBytes = $gbk.GetBytes($full)
[System.IO.File]::WriteAllBytes($path, $outBytes)
Write-Host "OK: Core\Src\main.c saved as GBK (CP936), USER CODE 2 block fixed."
