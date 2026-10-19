<#
.SYNOPSIS
  Install the pinned reconstruction tools listed in tool-pins.json.

.DESCRIPTION
  archive : download, verify sha256, extract to <ToolsDir>/<name>/<version>/
  npm     : npm install --prefix <ToolsDir>/<name>/<version> <package>@<version>
  cargo   : cargo install --git <repo> --rev <rev> --locked --root <ToolsDir>/<name>/<version> <package>
  uv      : uv sync --project <node>/<project>
  docker  : docker pull <image> (or docker build from a pinned commit), only when
            docker is reachable (natively or inside the WSL distribution)
  Every installed tool is probed afterwards and reported OK or FAIL.

.EXAMPLE
  ./fetch-tools.ps1 -List
  ./fetch-tools.ps1 -Tool colmap,brush
  ./fetch-tools.ps1 -Tool splat-transform -Force
#>
[CmdletBinding()]
param(
    [switch]$List,
    [string[]]$Tool,
    [switch]$Force,
    [string]$ToolsDir
)

# Cmdlet failures stop the script; native tools are judged by exit code only
# (Windows PowerShell 5 turns redirected native stderr into error records).
$ErrorActionPreference = 'Continue'
$PSDefaultParameterValues = @{ '*:ErrorAction' = 'Stop' }
$ProgressPreference = 'SilentlyContinue'

$ScriptDir = $PSScriptRoot
$NodeDir = Split-Path -Parent $ScriptDir
if (-not $ToolsDir) { $ToolsDir = Join-Path $ScriptDir 'cache' }
$WslDistro = 'Ubuntu-24.04'
$Pins = Get-Content -Raw -Path (Join-Path $ScriptDir 'tool-pins.json') | ConvertFrom-Json

function Get-PlatformKey {
    $arch = 'x86_64'
    $os = 'windows'
    try {
        $ri = [System.Runtime.InteropServices.RuntimeInformation]
        if ($ri::OSArchitecture -eq [System.Runtime.InteropServices.Architecture]::Arm64) { $arch = 'aarch64' }
        if ($ri::IsOSPlatform([System.Runtime.InteropServices.OSPlatform]::Linux)) { $os = 'linux' }
        elseif ($ri::IsOSPlatform([System.Runtime.InteropServices.OSPlatform]::OSX)) { $os = 'macos' }
    } catch {
        if ($env:PROCESSOR_ARCHITECTURE -eq 'ARM64') { $arch = 'aarch64' }
    }
    return "$arch-$os"
}
$Platform = Get-PlatformKey
$IsWin = $Platform.EndsWith('-windows')

function Get-Prop($obj, [string]$name) {
    if ($null -eq $obj) { return $null }
    $p = $obj.PSObject.Properties[$name]
    if ($null -eq $p) { return $null }
    return $p.Value
}

function Get-PlatformPin($pin) { return Get-Prop (Get-Prop $pin 'platforms') $Platform }

function Get-InstallDir([string]$name, $pin) { return Join-Path (Join-Path $ToolsDir $name) $pin.version }

function Get-BinPath([string]$name, $pin) {
    $bin = Get-Prop (Get-PlatformPin $pin) 'bin'
    if (-not $bin) { $bin = Get-Prop $pin 'bin' }
    if (-not $bin) { return $null }
    return Join-Path (Get-InstallDir $name $pin) $bin
}

function Find-Uv {
    $c = Get-Command uv -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    foreach ($p in @("$HOME/.local/bin/uv.exe", "$HOME/.local/bin/uv", "$HOME/.cargo/bin/uv.exe")) {
        if (Test-Path $p) { return $p }
    }
    return $null
}

# Returns the command prefix that reaches a docker daemon, or $null.
$script:DockerCmd = $false
function Get-DockerCmd {
    if ($script:DockerCmd -ne $false) { return $script:DockerCmd }
    $script:DockerCmd = $null
    if (Get-Command docker -ErrorAction SilentlyContinue) {
        & docker version *> $null
        if ($LASTEXITCODE -eq 0) { $script:DockerCmd = @('docker'); return $script:DockerCmd }
    }
    if ($IsWin -and (Get-Command wsl -ErrorAction SilentlyContinue)) {
        & wsl -d $WslDistro -- docker version *> $null
        if ($LASTEXITCODE -eq 0) { $script:DockerCmd = @('wsl', '-d', $WslDistro, '--', 'docker') }
    }
    return $script:DockerCmd
}

function Invoke-Docker([string[]]$dockerArgs, [switch]$Quiet) {
    $cmd = Get-DockerCmd
    $exe = $cmd[0]
    $all = @()
    if ($cmd.Count -gt 1) { $all += $cmd[1..($cmd.Count - 1)] }
    $all += $dockerArgs
    if ($Quiet) { & $exe @all *> $null } else { & $exe @all | Out-Host }
    return ($LASTEXITCODE -eq 0)
}

function Get-DockerRef($pin) {
    $build = Get-Prop $pin 'build'
    if ($build) { return (Get-Prop $build 'tag') }
    return (Get-Prop $pin 'image')
}

function Test-Installed([string]$name, $pin) {
    switch ($pin.kind) {
        'uv' { return (Test-Path (Join-Path (Join-Path $NodeDir $pin.project) '.venv')) }
        'docker' {
            if (-not (Get-DockerCmd)) { return $null }
            return (Invoke-Docker @('image', 'inspect', (Get-DockerRef $pin)) -Quiet)
        }
        default {
            $bin = Get-BinPath $name $pin
            if (-not $bin) { return $false }
            return (Test-Path $bin)
        }
    }
}

function Get-PlatformLabel($pin) {
    switch ($pin.kind) {
        'docker' { return 'linux/amd64 container' }
        'npm' { return 'any (node)' }
        'uv' { return 'any (uv)' }
        'cargo' { return 'any (cargo)' }
        default {
            if (Get-PlatformPin $pin) { return $Platform }
            return "$Platform unavailable"
        }
    }
}

function Show-List {
    $rows = foreach ($prop in $Pins.tools.PSObject.Properties) {
        $pin = $prop.Value
        $state = Test-Installed $prop.Name $pin
        if ($null -eq $state) { $label = 'unknown (docker unavailable)' }
        elseif ($state) { $label = 'installed' }
        else { $label = 'missing' }
        [pscustomobject]@{
            Name = $prop.Name; Version = $pin.version; Kind = $pin.kind
            Platform = (Get-PlatformLabel $pin); State = $label
        }
    }
    $rows | Format-Table -AutoSize | Out-String -Width 200 | Write-Output
}

function Save-Download([string]$url, [string]$dest) {
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dest) | Out-Null
    $curl = Get-Command curl.exe -ErrorAction SilentlyContinue
    if (-not $curl) { $curl = Get-Command curl -CommandType Application -ErrorAction SilentlyContinue }
    if ($curl) {
        & $curl.Source -L --fail --retry 3 -C - -o $dest $url | Out-Host
        if ($LASTEXITCODE -ne 0) { throw "download failed: $url" }
    } else {
        Invoke-WebRequest -Uri $url -OutFile $dest -UseBasicParsing
    }
}

function Expand-Into([string]$file, [string]$kind, [string]$dest) {
    New-Item -ItemType Directory -Force -Path $dest | Out-Null
    switch ($kind) {
        'exe' { Copy-Item $file (Join-Path $dest (Split-Path -Leaf $file)); return }
        '7z' {
            $sz = Get-Command 7z -ErrorAction SilentlyContinue
            if (-not $sz) { throw '7z archive needs 7z on PATH' }
            & $sz.Source x -y "-o$dest" $file | Out-Null
        }
        default {
            # The Windows bsdtar reads zip, tar.gz and tar.xz; a GNU tar earlier
            # on PATH (Git for Windows) cannot read zip.
            $tar = $null
            if ($IsWin -and (Test-Path "$env:SystemRoot\System32\tar.exe")) { $tar = "$env:SystemRoot\System32\tar.exe" }
            else {
                $c = Get-Command tar -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
                if ($c) { $tar = $c.Source }
            }
            if ($tar) {
                & $tar -xf $file -C $dest | Out-Host
                if ($LASTEXITCODE -eq 0) { return }
                if ($kind -ne 'zip') { throw "tar could not extract $file" }
            }
            if ($kind -ne 'zip') { throw "no extractor for $kind" }
            Expand-Archive -Path $file -DestinationPath $dest -Force
            return
        }
    }
    if ($LASTEXITCODE -ne 0) { throw "extract failed: $file" }
}

function Install-Archive([string]$name, $pin) {
    $pp = Get-PlatformPin $pin
    if (-not $pp -or -not (Get-Prop $pp 'url')) { Write-Host "  skipped (no $Platform build pinned)"; return $false }
    $dir = Get-InstallDir $name $pin
    $leaf = Split-Path -Leaf ([uri]$pp.url).AbsolutePath
    $dl = Join-Path (Join-Path $ToolsDir '_downloads') "$name-$($pin.version)-$leaf"
    if ($Force -or -not (Test-Path $dl) -or ((Get-FileHash -Algorithm SHA256 $dl).Hash.ToLower() -ne $pp.sha256)) {
        Write-Host "  downloading $($pp.url)"
        if (Test-Path $dl) { Remove-Item -Force $dl }
        Save-Download $pp.url $dl
    }
    $hash = (Get-FileHash -Algorithm SHA256 $dl).Hash.ToLower()
    if ($hash -ne $pp.sha256.ToLower()) {
        Remove-Item -Force $dl
        throw "sha256 mismatch for ${name}: got $hash, pinned $($pp.sha256)"
    }
    Write-Host "  sha256 OK"
    $tmp = "$dir.partial"
    if (Test-Path $tmp) { Remove-Item -Recurse -Force $tmp }
    Expand-Into $dl $pp.archive $tmp
    $inner = Get-Prop $pp 'inner'
    if ($inner) {
        $innerFile = Join-Path $tmp $inner
        Expand-Into $innerFile 'zip' $tmp
        Remove-Item -Force $innerFile
    }
    if (Test-Path $dir) { Remove-Item -Recurse -Force $dir }
    Move-Item $tmp $dir
    return $true
}

function Install-Npm([string]$name, $pin) {
    if (-not (Get-Command npm -ErrorAction SilentlyContinue)) { Write-Host '  skipped (npm unavailable)'; return $false }
    $dir = Get-InstallDir $name $pin
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    & npm install --prefix $dir --no-audit --no-fund --loglevel=error "$($pin.package)@$($pin.version)" | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "npm install failed for $name" }
    return $true
}

function Install-Cargo([string]$name, $pin) {
    if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) { Write-Host '  skipped (cargo unavailable)'; return $false }
    $c = $pin.cargo
    $dir = Get-InstallDir $name $pin
    $cargoArgs = @('install', '--git', $c.repo, '--rev', $c.rev, '--locked', '--root', $dir, $c.package)
    if ($Force) { $cargoArgs += '--force' }
    & cargo @cargoArgs | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "cargo install failed for $name" }
    return $true
}

function Install-Uv([string]$name, $pin) {
    $uv = Find-Uv
    if (-not $uv) { Write-Host '  skipped (uv unavailable)'; return $false }
    & $uv sync --project (Join-Path $NodeDir $pin.project) | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "uv sync failed for $name" }
    return $true
}

function Install-Docker([string]$name, $pin) {
    if (-not (Get-DockerCmd)) { Write-Host '  skipped (docker unavailable)'; return $false }
    $build = Get-Prop $pin 'build'
    if ($build) {
        $ctx = "$($build.repo).git#$($build.commit)"
        $ok = Invoke-Docker @('build', '-t', $build.tag, '-f', $build.dockerfile, $ctx)
    } else {
        $ok = Invoke-Docker @('pull', $pin.image)
    }
    if (-not $ok) { throw "docker step failed for $name" }
    return $true
}

function Invoke-Probe([string]$name, $pin) {
    $probe = @()
    if (Get-Prop $pin 'probe') { $probe = @($pin.probe) }
    switch ($pin.kind) {
        'uv' {
            $uv = Find-Uv
            $proj = Join-Path $NodeDir $pin.project
            Push-Location $proj
            try { & $uv run --project $proj @probe *> $null } finally { Pop-Location }
            return ($LASTEXITCODE -eq 0)
        }
        'docker' { return (Invoke-Docker @('image', 'inspect', (Get-DockerRef $pin)) -Quiet) }
        default {
            $bin = Get-BinPath $name $pin
            if (-not $bin -or -not (Test-Path $bin)) { return $false }
            Push-Location (Get-InstallDir $name $pin)
            try { & $bin @probe *> $null } finally { Pop-Location }
            return ($LASTEXITCODE -eq 0)
        }
    }
}

if ($List) { Show-List; exit 0 }

$names = @($Pins.tools.PSObject.Properties | ForEach-Object { $_.Name })
if ($Tool) {
    $selected = @($Tool | ForEach-Object { $_ -split ',' } | Where-Object { $_ })
    foreach ($t in $selected) { if ($names -notcontains $t) { throw "unknown tool: $t (known: $($names -join ', '))" } }
    $names = $selected
}

$failed = 0
foreach ($name in $names) {
    $pin = Get-Prop $Pins.tools $name
    Write-Host "== $name $($pin.version) [$($pin.kind)]"
    try {
        $installed = Test-Installed $name $pin
        if ($installed -and -not $Force -and $pin.kind -ne 'uv') {
            Write-Host '  already installed'
        } else {
            $done = switch ($pin.kind) {
                'archive' { Install-Archive $name $pin }
                'npm' { Install-Npm $name $pin }
                'cargo' { Install-Cargo $name $pin }
                'uv' { Install-Uv $name $pin }
                'docker' { Install-Docker $name $pin }
                default { throw "unknown kind $($pin.kind)" }
            }
            if (-not $done) { continue }
        }
        if (Invoke-Probe $name $pin) { Write-Host "  probe OK" }
        else { Write-Host "  probe FAIL"; $failed++ }
    } catch {
        Write-Host "  FAIL: $($_.Exception.Message)"
        $failed++
    }
}
if ($failed -gt 0) { exit 1 }
exit 0
