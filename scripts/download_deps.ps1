# PACER 依赖下载脚本 (Windows)
# 从 GitHub 拉取 STM32H7 HAL/CMSIS + FreeRTOS，复制到项目目录
# 用法: powershell -ExecutionPolicy Bypass -File scripts\download_deps.ps1

$ErrorActionPreference = "Stop"
$Root = Split-Path $PSScriptRoot -Parent
$Deps = Join-Path (Split-Path $Root -Parent) "_deps"

function Ensure-Repo($name, $url) {
    $path = Join-Path $Deps $name
    if (-not (Test-Path (Join-Path $path ".git"))) {
        New-Item -ItemType Directory -Force -Path $Deps | Out-Null
        git clone --depth 1 $url $path
    }
    return $path
}

Write-Host "=== PACER dependency download ==="
$cmsis = Ensure-Repo "cmsis_device_h7" "https://github.com/STMicroelectronics/cmsis_device_h7.git"
$hal   = Ensure-Repo "stm32h7xx_hal_driver" "https://github.com/STMicroelectronics/stm32h7xx_hal_driver.git"
$rtos  = Ensure-Repo "FreeRTOS-Kernel" "https://github.com/FreeRTOS/FreeRTOS-Kernel.git"

$dirs = @(
    "$Root\stm32hal\cmsis_device\Include",
    "$Root\stm32hal\hal\Inc",
    "$Root\stm32hal\hal\Src"
)
foreach ($d in $dirs) { New-Item -ItemType Directory -Force -Path $d | Out-Null }

Copy-Item "$cmsis\Include\*" "$Root\stm32hal\cmsis_device\Include\" -Recurse -Force
Copy-Item "$hal\Inc\*" "$Root\stm32hal\hal\Inc\" -Recurse -Force
Copy-Item "$hal\Src\*" "$Root\stm32hal\hal\Src\" -Recurse -Force

if (Test-Path "$cmsis\Source\Templates\gcc\startup_stm32h743xx.s") {
    Copy-Item "$cmsis\Source\Templates\gcc\startup_stm32h743xx.s" "$Root\cmake\startup_stm32h743xx.s" -Force
}

Get-ChildItem "$Root\freertos" -Exclude "FreeRTOSConfig.h" | Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
Copy-Item "$rtos\include" "$Root\freertos\include" -Recurse -Force
Copy-Item "$rtos\portable" "$Root\freertos\portable" -Recurse -Force
Copy-Item "$rtos\tasks.c","$rtos\queue.c","$rtos\list.c","$rtos\timers.c","$rtos\event_groups.c","$rtos\stream_buffer.c" "$Root\freertos\" -Force

# hal_conf.h
$template = "$hal\Inc\stm32h7xx_hal_conf_template.h"
$keep = @(
    'HAL_MODULE_ENABLED','HAL_CORTEX_MODULE_ENABLED','HAL_RCC_MODULE_ENABLED',
    'HAL_GPIO_MODULE_ENABLED','HAL_I2C_MODULE_ENABLED','HAL_TIM_MODULE_ENABLED',
    'HAL_UART_MODULE_ENABLED','HAL_FLASH_MODULE_ENABLED','HAL_PWR_MODULE_ENABLED',
    'HAL_DMA_MODULE_ENABLED','HAL_EXTI_MODULE_ENABLED'
)
$lines = Get-Content $template
$new = foreach ($line in $lines) {
    if ($line -match '^#define (HAL_[A-Z0-9_]+_MODULE_ENABLED)') {
        if ($keep -contains $Matches[1]) { $line } else { "/* $line */" }
    } elseif ($line -match '^#define\s+USE_RTOS\s+') {
        '#define  USE_RTOS                     0'
    } else { $line }
}
Set-Content -Path "$Root\include\stm32h7xx_hal_conf.h" -Value $new -Encoding UTF8

Write-Host "Done."
Write-Host "  HAL i2c: $(Test-Path '$Root\stm32hal\hal\Src\stm32h7xx_hal_i2c.c')"
Write-Host "  FreeRTOS: $(Test-Path '$Root\freertos\tasks.c')"
