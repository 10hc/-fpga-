# ============================================================
# prebuild_check.ps1 —— TD 综合前的自检（每次改完代码、点"综合"之前跑一次）
#
# 检查三件事（每一件都真实坑过我们）：
#   1) td_project\import\ 里的副本是否和源文件一致
#      —— TD 只编译副本！改了源文件不同步副本，板子上就还是旧行为
#         （key_short_long.v、wav_audio.v 就是这种"副本"文件）
#   2) 是否有"同一信号被多处驱动"（综合才报错，ModelSim 不报）
#   3) 工程清单 .al 里引用的文件是否都存在
#
# 用法：右键"使用 PowerShell 运行"，或
#       powershell -ExecutionPolicy Bypass -File prebuild_check.ps1
# ============================================================
[CmdletBinding()]
param(
    [string]$ProjectRoot = "",
    [string]$Python      = "python"
)

# $PSScriptRoot 在 param 默认值里取不到，必须在脚本体内算
if ([string]::IsNullOrEmpty($ProjectRoot)) {
    $ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
}

$ErrorActionPreference = "Continue"
$fail = 0

# 让 python 输出的中文在控制台里正常显示（否则按 GBK 解码成乱码）
try { [Console]::OutputEncoding = [Text.UTF8Encoding]::new($false) } catch { }
$env:PYTHONIOENCODING = "utf-8"

function Say-Ok   ($m) { Write-Host ("  [OK]   " + $m) -ForegroundColor Green }
function Say-Warn ($m) { Write-Host ("  [WARN] " + $m) -ForegroundColor Yellow }
function Say-Bad  ($m) { Write-Host ("  [BAD]  " + $m) -ForegroundColor Red; $script:fail++ }

Write-Host ""
Write-Host "== TD 综合前自检 ==" -ForegroundColor Cyan
Write-Host ("工程目录: " + $ProjectRoot)
Write-Host ""

$srcRoot = Join-Path $ProjectRoot "src"
$tdDir   = Join-Path $srcRoot "td_project"
$impDir  = Join-Path $tdDir "import"
$hdlDir  = Join-Path $srcRoot "user_source\hdl_source"

$alFiles = @(Get-ChildItem -Path $tdDir -Filter "*.al" -ErrorAction SilentlyContinue)
if ($alFiles.Count -eq 0) { Say-Bad ("在 " + $tdDir + " 找不到 .al 工程清单"); exit 1 }
$al = $alFiles[0].FullName
Write-Host ("清单: " + $al)
Write-Host ""

# ---------- 1) import 副本同步 ----------
Write-Host "[1/3] 检查 td_project\import 副本（TD 只编译这里的副本）" -ForegroundColor Cyan
if (Test-Path $impDir) {
    $copies = @(Get-ChildItem $impDir -Filter *.v -ErrorAction SilentlyContinue)
    if ($copies.Count -eq 0) { Say-Ok "import 目录为空（没有副本文件）" }
    foreach ($c in $copies) {
        # 必须用两层 @() 强制成数组：管道只回一个字符串时 PowerShell 会拆成
        # 字符数组，$cand[0] 就变成 'D' 这种单字符路径（真实踩过）
        $cand = @(@(
            (Join-Path $hdlDir $c.Name),
            (Join-Path $hdlDir ("SD\" + $c.Name))
        ) | Where-Object { Test-Path $_ })
        if ($cand.Count -eq 0 -or -not $cand[0]) {
            Say-Warn ($c.Name + " 在源目录找不到同名文件，请人工确认"); continue
        }
        $src = [string]$cand[0]
        $h1 = (Get-FileHash $c.FullName -Algorithm SHA256).Hash
        $h2 = (Get-FileHash $src -Algorithm SHA256).Hash
        if ($h1 -eq $h2) {
            Say-Ok ($c.Name + " 与源文件一致")
        } else {
            Copy-Item -LiteralPath $src -Destination $c.FullName -Force
            Say-Warn ($c.Name + " 与源文件不一致 → 已用源文件覆盖副本（TD 之前编译的是旧副本！）")
        }
    }
} else {
    Say-Ok "没有 import 目录（全部直接引用源文件）"
}
Write-Host ""

# ---------- 2) 多驱动扫描 ----------
Write-Host "[2/3] 扫描多驱动（综合会报 ERROR: constantly driven from multiple places）" -ForegroundColor Cyan
$scan = Join-Path $PSScriptRoot "scan_multi_driver.py"
if ((Test-Path $scan) -and (Get-Command $Python -ErrorAction SilentlyContinue)) {
    $out = & $Python $scan $hdlDir $impDir (Join-Path $tdDir "al_ip") 2>&1
    $out | ForEach-Object { Write-Host ("  " + $_) }
    if (($out | Select-String -Pattern "发现 0 处").Count -eq 0) { $fail++ }
} else {
    Say-Warn ("跳过：需要 python（当前找不到 '" + $Python + "'）或脚本缺失")
}
Write-Host ""

# ---------- 3) 清单引用完整性 ----------
Write-Host "[3/3] 检查 .al 清单里引用的文件是否都存在" -ForegroundColor Cyan
# 注意：TD 写的 .al 不是严格 XML（属性名里有裸 & 号，如 UsedInP&R），
#       用 [xml] 解析会报 "is an unexpected token"，所以这里只用正则取路径。
$raw = Get-Content $al -Raw
$paths = @([regex]::Matches($raw, '<File\s+Path="([^"]+)"') | ForEach-Object { $_.Groups[1].Value })
$missing = 0
foreach ($p in $paths) {
    $full = Join-Path $tdDir $p
    if (-not (Test-Path $full)) { Say-Bad ("清单引用但文件不存在: " + $p); $missing++ }
}
if ($missing -eq 0) { Say-Ok ("清单共 " + $paths.Count + " 个文件，全部存在") }
Write-Host ""

# ---------- 结论 ----------
if ($fail -eq 0) {
    Write-Host "== 结论：可以点『综合』了 ==" -ForegroundColor Green
    Write-Host "提醒：改过代码后必须让 TD 重新『综合』（只做布局布线会沿用旧网表）" -ForegroundColor Yellow
    Write-Host "提醒：如果 TD 正开着 video_player.v 等文件，先关掉再打开，避免把旧内容存回去" -ForegroundColor Yellow
} else {
    Write-Host ("== 结论：有 " + $fail + " 个问题，先处理再综合 ==") -ForegroundColor Red
}
Write-Host ""
exit $fail