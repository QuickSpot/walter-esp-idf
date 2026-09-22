<#
.SYNOPSIS
    Builds, flashes and runs the walter-esp-idf examples on a Walter board and
    reports PASS/FAIL for each.

.DESCRIPTION
    Each example is copied to a work directory and patched there, so the
    repository is never written to and there is nothing to clean. The verdict
    comes from matching the serial log against Criteria.psd1: every whitelist
    marker must appear, any blacklist marker fails immediately.

    This window shows the verdicts. Each example gets its own console window
    running its build, flash and monitor, left open afterwards so the whole
    scrollback stays readable - and a real console is needed regardless,
    because esp_idf_monitor refuses to start unless stdin is a TTY.

    Answers are remembered between runs; see -Fresh.

.PARAMETER IdfVersion
    EIM installation name, e.g. v5.5.2. Prompted when omitted.

.PARAMETER IdfPath
    Path to an esp-idf checkout, used instead of an EIM installation.

.PARAMETER Port
    Serial port of the board under test, e.g. COM3. Prompted when omitted.

.PARAMETER Examples
    Examples to run; 'all' selects every supported one. Prompted when omitted.

.PARAMETER Apn
    Cellular APN. Pass an empty string to leave it network-assigned.

.PARAMETER LogLevel
    INFO (default), DEBUG or VERBOSE. DEBUG and above compile in the driver's
    AT-command trace, which is a lot of output.

.PARAMETER TimeoutSec
    Wall-clock budget per example, default 300. Criteria.psd1 raises it for the
    slow ones unless -StrictTimeout is given.

.PARAMETER NoTimeout
    Never fail an example on the wall-clock budget.

.PARAMETER StrictTimeout
    Use -TimeoutSec and -QuietSec as given, ignoring per-example overrides.

.PARAMETER QuietSec
    Fail an example after this many seconds with no serial output, default 120.
    Several examples return out of app_main on failure and then go silent
    forever; without this they would burn the full timeout.

.PARAMETER BuildTimeoutSec
    Budget for one build, default 1800.

.PARAMETER FlashTimeoutSec
    Budget for one flash, default 300.

.PARAMETER WorkRoot
    Where sandboxes and logs go. Default %TEMP%\walter-tests.

.PARAMETER StageOnly
    Stage and patch the examples, then stop. No IDF, no board needed.

.PARAMETER NoFlash
    Build only. No board needed.

.PARAMETER CloseWindows
    Close each example's window when it finishes instead of leaving it open.
    Implies -NoRetry, since a window that closes itself cannot be closed to
    decline a retry.

.PARAMETER NoRetry
    Fail an example as soon as a phase fails instead of offering a retry in its
    window. Use this for unattended runs - the retry prompt waits indefinitely.

.PARAMETER ReplayLog
    Run a captured monitor log through the matcher and print the verdict.
    Needs -ReplayExample. No IDF, no board needed.

.PARAMETER ReplayExample
    Which example's criteria -ReplayLog should apply.

.PARAMETER Yes
    Take the remembered or default answer for every prompt and do not ask for
    confirmation. The ESP-IDF version and the board are only auto-selected when
    there is exactly one candidate.

.PARAMETER Fresh
    Ignore the answers remembered from the last run.

.EXAMPLE
    .\Run.ps1

.EXAMPLE
    .\Run.ps1 -IdfVersion v5.5.2 -Port COM3 -Examples tcp,udp -Apn '' -Yes
#>
[CmdletBinding()]
param(
    [string]   $IdfVersion,
    [string]   $IdfPath,
    [string]   $Port,
    [string[]] $Examples,
    [string]   $Apn,
    [ValidateSet('INFO', 'DEBUG', 'VERBOSE')]
    [string]   $LogLevel,
    [int]      $TimeoutSec = 300,
    [switch]   $NoTimeout,
    [switch]   $StrictTimeout,
    [int]      $QuietSec = 120,
    [int]      $BuildTimeoutSec = 1800,
    [int]      $FlashTimeoutSec = 300,
    [string]   $WorkRoot,
    [switch]   $StageOnly,
    [switch]   $NoFlash,
    [switch]   $CloseWindows,
    [switch]   $NoRetry,
    [string]   $ReplayLog,
    [string]   $ReplayExample,
    [switch]   $Yes,
    [switch]   $Fresh
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir '..') | Select-Object -ExpandProperty Path
$CriteriaFile = Join-Path $ScriptDir 'Criteria.psd1'
$WorkerScript = Join-Path $ScriptDir 'internal\Worker.ps1'

$SupportedExamples = @('bluecherry', 'coap', 'http', 'https', 'mqtt', 'mqtts', 'positioning', 'tcp', 'udp')

try { $Host.UI.RawUI.WindowTitle = 'Walter tests - controller' } catch { }

# ===========================================================================
# Small output helpers
# ===========================================================================

function Write-Head {
    param([string] $Text)
    Write-Host ''
    Write-Host "== $Text " -ForegroundColor Cyan -NoNewline
    Write-Host ('=' * [Math]::Max(1, 74 - $Text.Length)) -ForegroundColor Cyan
}

function Write-Info { param([string] $m) Write-Host "   $m" }
function Write-Note { param([string] $m) Write-Host "   $m" -ForegroundColor DarkGray }
function Write-Warn { param([string] $m) Write-Host "   $m" -ForegroundColor Yellow }
function Write-Bad { param([string] $m) Write-Host "   $m" -ForegroundColor Red }
function Write-Good { param([string] $m) Write-Host "   $m" -ForegroundColor Green }

# A CR-updated status line turns into megabytes of noise when redirected to a
# file, so fall back to plain lines at a much lower rate there.
$LiveStatus = $true
try { $LiveStatus = -not [Console]::IsOutputRedirected } catch { }
$StatusEverySec = if ($LiveStatus) { 2 } else { 20 }

function Write-Status {
    param([string] $Text)
    if ($LiveStatus) { Write-Host ("`r   {0,-76}" -f $Text) -NoNewline }
    else { Write-Host "   $Text" }
}

function Clear-Status {
    if ($LiveStatus) { Write-Host ("`r{0}`r" -f (' ' * 79)) -NoNewline }
}

# ===========================================================================
# Remembered answers
#
# Every prompt defaults to last run's answer. Kept in the user profile so it
# never shows up in git status, and never fatal: an unreadable file just means
# answering the prompts again.
# ===========================================================================

$SettingsPath = Join-Path $env:LOCALAPPDATA 'walter-esp-idf\tests-last-run.json'

function Get-SavedAnswers {
    if ($Fresh) { return @{} }
    if (-not (Test-Path -LiteralPath $SettingsPath)) { return @{} }
    try {
        $json = Get-Content -LiteralPath $SettingsPath -Raw | ConvertFrom-Json
        $saved = @{}
        foreach ($p in $json.PSObject.Properties) { $saved[$p.Name] = $p.Value }
        return $saved
    } catch {
        Write-Note "ignoring unreadable $SettingsPath"
        return @{}
    }
}

function Save-Answers {
    param([hashtable] $Answers)
    try {
        $dir = Split-Path $SettingsPath -Parent
        if (-not (Test-Path -LiteralPath $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
        Write-TextFile -Path $SettingsPath -Text ([pscustomobject] $Answers | ConvertTo-Json -Depth 4)
    } catch {
        Write-Note "could not remember your answers: $($_.Exception.Message)"
    }
}

function Get-SavedValue {
    param([string] $Key, $Fallback = $null)
    if ($Saved.ContainsKey($Key) -and $null -ne $Saved[$Key]) { return $Saved[$Key] }
    return $Fallback
}

function Write-TextFile {
    # UTF-8 without BOM, LF endings - PowerShell's own defaults are neither.
    param([string] $Path, [string] $Text)
    $text = $Text -replace "`r`n", "`n"
    [System.IO.File]::WriteAllText($Path, $text, (New-Object System.Text.UTF8Encoding($false)))
}

# ===========================================================================
# Prompts. Every one has a matching parameter, so a run can skip the lot.
# ===========================================================================

function Read-Choice {
    param(
        [string] $Title,
        [object[]] $Items,
        [scriptblock] $Label,
        [int] $DefaultIndex = 0,
        [string] $ManualPrompt
    )
    while ($true) {
        Write-Host ''
        Write-Host "   $Title" -ForegroundColor White
        for ($i = 0; $i -lt $Items.Count; $i++) {
            $mark = if ($i -eq $DefaultIndex) { '*' } else { ' ' }
            Write-Host ("    $mark [{0}] {1}" -f ($i + 1), (& $Label $Items[$i]))
        }
        if ($ManualPrompt) { Write-Host "      [m] $ManualPrompt" }
        $ans = Read-Host "   Choice (Enter = default)"
        if ([string]::IsNullOrWhiteSpace($ans)) { return $Items[$DefaultIndex] }
        if ($ManualPrompt -and $ans.Trim().ToLower() -eq 'm') { return $null }
        $n = 0
        if ([int]::TryParse($ans.Trim(), [ref] $n) -and $n -ge 1 -and $n -le $Items.Count) {
            return $Items[$n - 1]
        }
        Write-Warn 'Not a valid choice.'
    }
}

function Read-MultiChoice {
    param([string] $Title, [string[]] $Items, [string[]] $Default)
    while ($true) {
        Write-Host ''
        Write-Host "   $Title" -ForegroundColor White
        for ($i = 0; $i -lt $Items.Count; $i++) {
            $mark = if ($Default -and $Default -contains $Items[$i]) { '*' } else { ' ' }
            Write-Host ("    {0} [{1,2}] {2}" -f $mark, ($i + 1), $Items[$i])
        }
        Write-Host '      [ a] all'
        if ($Default) { Write-Host "      * = last time; Enter keeps $($Default -join ', ')" -ForegroundColor DarkGray }
        $ans = Read-Host '   Numbers separated by space or comma, or "a"'
        if ([string]::IsNullOrWhiteSpace($ans)) {
            if ($Default) { return $Default }
            Write-Warn 'Pick at least one.'
            continue
        }
        if ($ans.Trim().ToLower() -eq 'a') { return $Items }
        $picked = @()
        $ok = $true
        foreach ($tok in ($ans -split '[,\s]+' | Where-Object { $_ })) {
            $n = 0
            if ([int]::TryParse($tok, [ref] $n) -and $n -ge 1 -and $n -le $Items.Count) {
                if ($picked -notcontains $Items[$n - 1]) { $picked += $Items[$n - 1] }
            } else {
                Write-Warn "Not a valid entry: $tok"
                $ok = $false
            }
        }
        if ($ok -and $picked.Count -gt 0) { return $picked }
    }
}

function Read-YesNo {
    param([string] $Question, [bool] $Default = $true)
    $hint = if ($Default) { 'Y/n' } else { 'y/N' }
    while ($true) {
        $ans = Read-Host "   $Question [$hint]"
        if ([string]::IsNullOrWhiteSpace($ans)) { return $Default }
        switch ($ans.Trim().ToLower()) {
            'y' { return $true }
            'yes' { return $true }
            'n' { return $false }
            'no' { return $false }
            default { Write-Warn 'Answer y or n.' }
        }
    }
}

# ===========================================================================
# ESP-IDF discovery
# ===========================================================================

function Get-IdfInstallations {
    $found = @()

    # The Espressif Installation Manager keeps a machine-readable registry.
    $eim = 'C:\Espressif\tools\eim_idf.json'
    if (Test-Path -LiteralPath $eim) {
        try {
            $j = Get-Content -LiteralPath $eim -Raw | ConvertFrom-Json
            foreach ($i in $j.idfInstalled) {
                if (Test-Path -LiteralPath (Join-Path $i.path 'tools\idf.py')) {
                    $found += [pscustomobject]@{
                        Name       = $i.name
                        IdfPath    = $i.path
                        Python     = $i.python
                        Activation = $i.activationScript
                        IsDefault  = ($i.id -eq $j.idfSelectedId)
                        Source     = 'eim_idf.json'
                    }
                }
            }
        } catch {
            Write-Warn "Could not parse $eim ($($_.Exception.Message)); falling back to a filesystem scan."
        }
    }

    if ($found.Count -eq 0) {
        foreach ($root in @('C:\esp', 'C:\Espressif\frameworks', "$env:USERPROFILE\esp")) {
            if (-not (Test-Path -LiteralPath $root)) { continue }
            foreach ($cand in @(
                    (Get-ChildItem -LiteralPath $root -Directory -ErrorAction SilentlyContinue | ForEach-Object { Join-Path $_.FullName 'esp-idf' })
                    (Get-ChildItem -LiteralPath $root -Directory -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName })
                )) {
                if (Test-Path -LiteralPath (Join-Path $cand 'tools\idf.py')) {
                    if ($found.IdfPath -contains $cand) { continue }
                    $found += [pscustomobject]@{
                        Name       = Split-Path (Split-Path $cand -Parent) -Leaf
                        IdfPath    = $cand
                        Python     = (Find-IdfPython -IdfPath $cand)
                        Activation = $null
                        IsDefault  = $false
                        Source     = 'filesystem scan'
                    }
                }
            }
        }
    }

    return @($found)
}

function Find-IdfPython {
    param([string] $IdfPath)
    $verDir = Split-Path (Split-Path $IdfPath -Parent) -Leaf
    foreach ($c in @(
            "C:\Espressif\tools\python\$verDir\venv\Scripts\python.exe"
            "$env:USERPROFILE\.espressif\python_env\*\Scripts\python.exe"
        )) {
        $hit = Get-Item -Path $c -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    $onPath = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }
    return $null
}

function Resolve-IdfEnvironment {
    <#
        Builds the environment the worker will apply.

        The EIM activation profile is invoked with -e, which only PRINTS the
        variables. Dot-sourcing it would also run its trailing
        `eim select <version>`, rewriting idfSelectedId in eim_idf.json and
        changing which IDF the VS Code extension picks.
    #>
    param($Install)

    $spec = [ordered]@{
        Name          = $Install.Name
        IdfPath       = $Install.IdfPath
        Python        = $Install.Python
        IdfPy         = (Join-Path $Install.IdfPath 'tools\idf.py')
        Vars          = @{}
        PathPrepend   = ''
        SourceProfile = $null
    }

    if ($Install.Activation -and (Test-Path -LiteralPath $Install.Activation)) {
        try {
            # 6>&1 because the profile emits most pairs through Write-Host, and
            # StrictMode off because it dereferences $null when eim is not on
            # PATH, which would otherwise become an error here.
            $lines = & { Set-StrictMode -Off; & $Install.Activation -e } 6>&1
            foreach ($l in $lines) {
                $t = [string]$l
                if ($t -match '^(?<k>[A-Za-z_][A-Za-z0-9_]*)=(?<v>.*)$') {
                    $k = $Matches['k']
                    $v = $Matches['v']
                    if ($k -eq 'PATH') {
                        $spec.PathPrepend = $v
                    } elseif ($k -eq 'SYSTEM_PATH') {
                        # The ambient PATH as it was when EIM generated the
                        # profile. The live one is more current; ignore this.
                    } else {
                        $spec.Vars[$k] = $v
                    }
                }
            }
        } catch {
            Write-Warn "Could not read the activation profile ($($_.Exception.Message))."
        }
    }

    if (-not $spec.Vars.ContainsKey('IDF_PATH')) { $spec.Vars['IDF_PATH'] = $Install.IdfPath }

    if (-not $spec.PathPrepend) {
        if ($Install.Activation -and (Test-Path -LiteralPath $Install.Activation)) {
            Write-Warn 'Activation profile produced no PATH; the worker will dot-source it instead.'
            $spec.SourceProfile = $Install.Activation
        } else {
            # Bare checkout: idf.py bootstraps most of this itself as long as
            # IDF_PATH and a usable python are set.
            Write-Warn 'No activation profile; relying on IDF_PATH and the discovered python.'
        }
    }

    return $spec
}

function Select-IdfInstallation {
    if ($IdfPath) {
        if (-not (Test-Path -LiteralPath (Join-Path $IdfPath 'tools\idf.py'))) {
            throw "-IdfPath '$IdfPath' does not contain tools\idf.py."
        }
        return [pscustomobject]@{
            Name = (Split-Path $IdfPath -Leaf); IdfPath = $IdfPath
            Python = (Find-IdfPython -IdfPath $IdfPath); Activation = $null
            IsDefault = $false; Source = '-IdfPath'
        }
    }

    $installs = @(Get-IdfInstallations)

    if ($IdfVersion) {
        $hit = $installs | Where-Object { $_.Name -eq $IdfVersion } | Select-Object -First 1
        if (-not $hit) { throw "No ESP-IDF installation named '$IdfVersion'. Found: $(($installs.Name) -join ', ')" }
        return $hit
    }

    if ($Yes -and $installs.Count -eq 1) { return $installs[0] }

    # The harness will not proceed without a real installation.
    while ($true) {
        if ($installs.Count -gt 0) {
            Write-Info "Found $($installs.Count) ESP-IDF installation(s) via $($installs[0].Source):"
            # Last run's choice wins, else v5.5.2 - what every example's
            # .vscode/settings.json and dependencies.lock records.
            $wanted = Get-SavedValue -Key 'IdfVersion' -Fallback 'v5.5.2'
            $preferred = 0
            for ($i = 0; $i -lt $installs.Count; $i++) {
                if ($installs[$i].Name -eq 'v5.5.2') { $preferred = $i }
            }
            for ($i = 0; $i -lt $installs.Count; $i++) {
                if ($installs[$i].Name -eq $wanted) { $preferred = $i }
            }
            $pick = Read-Choice -Title 'Which ESP-IDF?' -Items $installs -DefaultIndex $preferred `
                -ManualPrompt 'enter a path manually' `
                -Label { param($x) "{0,-10} {1}{2}" -f $x.Name, $x.IdfPath, $(if ($x.IsDefault) { '  (EIM default)' } else { '' }) }
            if ($pick) { return $pick }
        } else {
            Write-Warn 'No ESP-IDF installation was detected automatically.'
        }

        $manual = Read-Host '   Path to an esp-idf checkout (the folder containing tools\idf.py)'
        if ($manual -and (Test-Path -LiteralPath (Join-Path $manual 'tools\idf.py'))) {
            return [pscustomobject]@{
                Name = (Split-Path $manual -Leaf); IdfPath = (Resolve-Path $manual).Path
                Python = (Find-IdfPython -IdfPath $manual); Activation = $null
                IsDefault = $false; Source = 'manual'
            }
        }
        Write-Bad "Not an ESP-IDF checkout: '$manual' (no tools\idf.py). Try again."
    }
}

# ===========================================================================
# Serial port discovery
# ===========================================================================

function Get-SerialPorts {
    $out = @()
    try {
        $dev = Get-CimInstance Win32_PnPEntity -ErrorAction Stop | Where-Object { $_.Name -match '\(COM\d+\)' }
        foreach ($d in $dev) {
            if ($d.Name -match '\((COM\d+)\)') {
                $out += [pscustomobject]@{
                    Port      = $Matches[1]
                    Name      = $d.Name
                    Espressif = ($d.DeviceID -like '*VID_303A*')   # Espressif USB JTAG/serial
                }
            }
        }
    } catch {
        Write-Warn "Could not enumerate serial ports: $($_.Exception.Message)"
    }
    return @($out | Sort-Object { [int]($_.Port -replace '\D', '') })
}

function Select-SerialPort {
    if ($Port) { return $Port }
    $ports = @(Get-SerialPorts)
    $esp = @($ports | Where-Object { $_.Espressif })
    # The @() is load-bearing: an if-expression unrolls a one-element array to
    # a bare object, which has no .Count under Set-StrictMode.
    $items = @(if ($esp.Count -gt 0) { $esp } else { $ports })

    if ($items.Count -eq 0) {
        Write-Warn 'No serial ports detected.'
        return (Read-Host '   Serial port (e.g. COM3)').Trim()
    }
    # -Yes only auto-picks when there is no ambiguity.
    if ($Yes -and $items.Count -eq 1) { return $items[0].Port }

    $wanted = Get-SavedValue -Key 'Port'
    $preferred = 0
    for ($i = 0; $i -lt $items.Count; $i++) {
        if ($items[$i].Port -eq $wanted) { $preferred = $i }
    }

    $pick = Read-Choice -Title 'Which board?' -Items $items -DefaultIndex $preferred `
        -ManualPrompt 'enter a port manually' `
        -Label { param($x) "{0,-6} {1}{2}" -f $x.Port, $x.Name, $(if ($x.Espressif) { '' } else { '  (not an Espressif device)' }) }
    if ($pick) { return $pick.Port }
    return (Read-Host '   Serial port (e.g. COM3)').Trim()
}

# ===========================================================================
# Staging and patching - all in the work directory; the repo is only read.
# ===========================================================================

function New-StagedExample {
    param([string] $Name, [string] $Dest)

    $src = Join-Path $RepoRoot "examples\$Name"
    if (-not (Test-Path -LiteralPath $src)) { throw "No such example: $src" }

    # A fresh copy is the fullclean: nothing stale to delete afterwards.
    $rc = @(
        $src, $Dest, '/E', '/NFL', '/NDL', '/NJH', '/NJS', '/NP', '/R:1', '/W:1'
        '/XD', 'build', 'managed_components', '.vscode'
        '/XF', 'sdkconfig', 'sdkconfig.old', 'dependencies.lock'
    )
    & robocopy.exe @rc | Out-Null
    if ($LASTEXITCODE -ge 8) { throw "robocopy failed with exit code $LASTEXITCODE copying $src" }
    $global:LASTEXITCODE = 0
}

function Set-LocalComponentOverride {
    <#
        Repoints the dependency at the local checkout, replacing the published
        dptechnics/walter-modem entry - the form the repo itself documents,
        with an absolute path because a relative one would be wrong from the
        work directory.

        Not `override_path` under the registry name: the component manager
        resolves that happily and CMake then dies with "Failed to resolve
        component 'dptechnics__walter-modem' required by component 'main'",
        because ESP-IDF names a component after its directory basename. The
        rename costs nothing - no example CMakeLists requires it by name.
    #>
    param([string] $ManifestPath, [string] $ComponentRoot)

    $text = [System.IO.File]::ReadAllText($ManifestPath)

    $idfReq = '>=5.0.0'
    if ($text -match '(?m)^\s*idf:\s*[''"]?([^''"\r\n]+)[''"]?\s*$') { $idfReq = $Matches[1].Trim() }

    $abs = $ComponentRoot -replace '\\', '/'
    $name = Split-Path $ComponentRoot -Leaf

    Write-TextFile -Path $ManifestPath -Text @"
# Generated by tests/Run.ps1 - do not commit.
# Builds against the local checkout instead of the published component.
dependencies:
  idf: "$idfReq"
  ${name}:
    path: "$abs"
"@
}

function Set-ExampleApn {
    <#
        Only bluecherry actually uses an APN: positioning defines CELLULAR_APN
        and ignores it, and the other seven call definePDPContext() with no
        arguments, which no define or -D flag can reach. Both shapes are
        patched and the report says which fired.
    #>
    param([string] $ProjectDir, [string] $ApnValue)

    $apn = $ApnValue
    $touched = @()
    $files = @(Get-ChildItem -LiteralPath (Join-Path $ProjectDir 'main') -File -Recurse |
            Where-Object { $_.Extension -eq '.c' -or $_.Extension -eq '.cpp' })

    foreach ($f in $files) {
        $text = [System.IO.File]::ReadAllText($f.FullName)
        $before = $text
        $what = @()

        # MatchEvaluator rather than a replacement string: '$' in the APN
        # would otherwise be read as a group reference.
        $text = [regex]::Replace($text, '(#define\s+CELLULAR_APN\s+)"[^"]*"', {
                param($m) $m.Groups[1].Value + '"' + $apn + '"'
            })
        if ($text -ne $before) { $what += 'CELLULAR_APN' }

        $mid = $text
        $text = [regex]::Replace($text, 'definePDPContext\s*\(\s*\)', {
                param($m) 'definePDPContext(1, "' + $apn + '")'
            })
        if ($text -ne $mid) { $what += 'definePDPContext()' }

        if ($text -ne $before) {
            [System.IO.File]::WriteAllText($f.FullName, $text, (New-Object System.Text.UTF8Encoding($false)))
            $touched += "$($f.Name): $($what -join ' + ')"
        }
    }
    return $touched
}

function Set-ExampleLogLevel {
    <#
        Rewrites the level in the copied sdkconfig.defaults.

        Only the DEFAULT_LEVEL choice is set: CONFIG_LOG_MAXIMUM_LEVEL_DEBUG
        "depends on LOG_DEFAULT_LEVEL < 4" so it is unselectable at DEBUG, and
        LOG_MAXIMUM_EQUALS_DEFAULT already drags the maximum along - which is
        what compiles the driver's ESP_LOGD AT trace in.
    #>
    param([string] $DefaultsPath, [string] $Level)

    $value = @{ INFO = 3; DEBUG = 4; VERBOSE = 5 }[$Level]

    $kept = @()
    if (Test-Path -LiteralPath $DefaultsPath) {
        foreach ($line in [System.IO.File]::ReadAllLines($DefaultsPath)) {
            if ($line -match '^\s*CONFIG_LOG_DEFAULT_LEVEL') { continue }
            if ($line -match '^\s*CONFIG_LOG_MAXIMUM_LEVEL') { continue }
            if ($line -match '^\s*CONFIG_LOG_MAXIMUM_EQUALS_DEFAULT') { continue }
            $kept += $line
        }
    }
    $kept += "CONFIG_LOG_DEFAULT_LEVEL_$Level=y"
    $kept += "CONFIG_LOG_DEFAULT_LEVEL=$value"

    Write-TextFile -Path $DefaultsPath -Text (($kept -join "`n") + "`n")
}

# ===========================================================================
# Worker control
# ===========================================================================

# Start-Process -ArgumentList joins array elements with a plain space and does
# NOT quote them, so any element containing one is torn in half.
function ConvertTo-CommandLine {
    param([string[]] $Arguments)
    $quoted = foreach ($a in $Arguments) {
        if ($a -match '[\s"]') { '"' + ($a -replace '"', '\"') + '"' } else { $a }
    }
    return ($quoted -join ' ')
}

function Start-ExampleWindow {
    <#
        One window per example, running its build, flash and monitor in
        sequence and left open afterwards so the whole scrollback stays
        readable.

        A plain console window on purpose: separate windows keep each
        example's output apart, which a split pane cannot.
    #>
    param(
        [string] $RunDir, [string] $EnvFile, [string] $Name,
        [string] $ProjDir, [string] $SerialPort, [bool] $SkipFlash
    )

    foreach ($stale in @('started', 'build.done', 'flash.done', 'monitor.done', 'waiting', 'retry', 'verdict')) {
        $p = Join-Path $RunDir "$Name.$stale"
        if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath $p -Force }
    }

    $wArgs = @()
    if (-not $CloseWindows) { $wArgs += '-NoExit' }
    $wArgs += @(
        '-NoProfile', '-ExecutionPolicy', 'Bypass',
        '-File', $WorkerScript,
        '-RunDir', $RunDir, '-EnvFile', $EnvFile,
        '-Name', $Name, '-ProjDir', $ProjDir
    )
    if ($SerialPort) { $wArgs += @('-Port', $SerialPort) }
    if ($SkipFlash) { $wArgs += '-NoFlash' }
    # A window that closes itself cannot be closed to decline a retry, so the
    # two go together.
    if ($NoRetry -or $CloseWindows) { $wArgs += '-NoRetry' }

    $proc = Start-Process -FilePath 'powershell.exe' `
        -ArgumentList (ConvertTo-CommandLine $wArgs) -PassThru -ErrorAction Stop

    $started = Join-Path $RunDir "$Name.started"
    for ($i = 0; $i -lt 80; $i++) {
        if (Test-Path -LiteralPath $started) { return $proc }
        if ($proc.HasExited) { throw "The $Name window exited immediately (exit $($proc.ExitCode))." }
        Start-Sleep -Milliseconds 250
    }
    throw "The $Name window did not start within 20s. Try running $WorkerScript by hand."
}

# A console app's console host is a CHILD of it, so killing a window's
# descendants indiscriminately kills the host, and the window vanishes taking
# the scrollback with it.
$ConsoleHostProcesses = @('conhost.exe', 'openconsole.exe', 'windowsterminal.exe')

# Stops the work inside an example's window - in practice the idf.py monitor -
# while leaving the window itself alive and readable.
function Stop-Descendants {
    param([int] $ParentId)
    $kids = @(Get-CimInstance Win32_Process -Filter "ParentProcessId=$ParentId" -ErrorAction SilentlyContinue)
    foreach ($k in $kids) {
        if ($ConsoleHostProcesses -contains $k.Name.ToLower()) { continue }
        Stop-Descendants -ParentId $k.ProcessId
        try { Stop-Process -Id $k.ProcessId -Force -ErrorAction Stop } catch { }
    }
}

function Wait-ForRetryDecision {
    <#
        After a failing verdict the window asks whether to flash and monitor
        again. Returns $true if it did, $false if the person gave up (closed
        the window) or was never asked (-NoRetry).

        The window signals a retry with <name>.retry; this consumes that and
        the verdict file so the next attempt starts clean.
    #>
    param([string] $RunDir, [string] $Name, $Proc)

    $retry = Join-Path $RunDir "$Name.retry"
    $verdict = Join-Path $RunDir "$Name.verdict"

    # Until the prompt appears there is a deadline; once it is up, wait as long
    # as it takes.
    $deadline = (Get-Date).AddSeconds(15)
    $announced = $false

    while ($true) {
        if (Test-Path -LiteralPath $retry) {
            Remove-Item -LiteralPath $retry -Force -ErrorAction SilentlyContinue
            Remove-Item -LiteralPath $verdict -Force -ErrorAction SilentlyContinue
            Clear-Status
            return $true
        }
        if ($Proc.HasExited) {
            Clear-Status
            return $false
        }
        if (Test-WindowWaiting -RunDir $RunDir -Name $Name) {
            if (-not $announced) {
                $announced = $true
                Write-Warn "did not pass - waiting for you in the $Name window (Enter to run again, close it to accept the failure)"
            }
        } elseif (-not $announced -and (Get-Date) -gt $deadline) {
            # No prompt ever appeared: -NoRetry, or the window is finishing.
            Clear-Status
            return $false
        }
        Start-Sleep -Milliseconds 400
    }
}

# The window is asking whether to retry, so nothing is hung.
function Test-WindowWaiting {
    param([string] $RunDir, [string] $Name)
    return (Test-Path -LiteralPath (Join-Path $RunDir "$Name.waiting"))
}

function Wait-Phase {
    <#
        Waits for <name>.<phase>.done and returns the exit code it holds.
        Throws on timeout, or if the window was closed before it got there -
        which is how someone declines a retry.

        The window writes .done only once the phase has settled, so a failure
        it is still offering to retry simply keeps this waiting, and the
        timeout is held while that prompt is up.
    #>
    param(
        [string] $RunDir, [string] $Name, [string] $Phase,
        $Proc, [int] $TimeoutSeconds, [string] $What
    )
    $done = Join-Path $RunDir "$Name.$Phase.done"
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $lastPrint = [DateTime]::MinValue
    $waiting = $false
    while ($true) {
        if (Test-Path -LiteralPath $done) {
            $raw = (Get-Content -LiteralPath $done -Raw).Trim()
            $code = 0
            if (-not [int]::TryParse($raw, [ref] $code)) { $code = 1 }
            Clear-Status
            Write-Info "$What ... done in $([int]$sw.Elapsed.TotalSeconds)s (exit $code)"
            return $code
        }
        if ($Proc.HasExited) {
            Clear-Status
            throw "the $Name window was closed during $What"
        }

        if (Test-WindowWaiting -RunDir $RunDir -Name $Name) {
            if (-not $waiting) {
                $waiting = $true
                Clear-Status
                Write-Warn "$What failed - waiting for you in the $Name window (Enter to retry, close it to give up)"
            }
            $sw.Restart()   # do not count thinking time against the timeout
        } else {
            if ($waiting) {
                $waiting = $false
                Write-Note "retrying $What"
            }
            if ($TimeoutSeconds -gt 0 -and $sw.Elapsed.TotalSeconds -gt $TimeoutSeconds) {
                Clear-Status
                throw "$What exceeded ${TimeoutSeconds}s."
            }
            if (((Get-Date) - $lastPrint).TotalSeconds -ge $StatusEverySec) {
                $lastPrint = Get-Date
                Write-Status ("{0} ... {1}s" -f $What, [int]$sw.Elapsed.TotalSeconds)
            }
        }
        Start-Sleep -Milliseconds 400
    }
}

# ===========================================================================
# Log follower and matcher
# ===========================================================================

$AnsiPattern = [regex]("$([char]27)\[[0-9;?]*[ -/]*[@-~]")

function Get-BuildFailureContext {
    <#
        The tail of a failed build log is CMake warnings and "ninja: build
        stopped", never the thing that broke. Pull out the compiler's own error
        lines instead, truncated: the line after a "FAILED:" is the whole gcc
        invocation, thousands of characters of -I flags.
    #>
    param([string] $LogPath, [int] $Max = 25)

    if (-not (Test-Path -LiteralPath $LogPath)) { return @() }
    $lines = [System.IO.File]::ReadAllLines($LogPath)

    $hits = New-Object System.Collections.Generic.List[string]
    foreach ($l in $lines) {
        if ($l -cmatch 'error:|fatal error|undefined reference|^FAILED:|^ninja: error') {
            $t = $l.TrimEnd()
            if ($t.Length -gt 200) { $t = $t.Substring(0, 197) + '...' }
            $hits.Add($t)
            if ($hits.Count -ge $Max) { break }
        }
    }
    if ($hits.Count -eq 0) { return @($lines | Select-Object -Last $Max) }
    return @($hits)
}

function New-LogFollower {
    param([string[]] $Paths)
    return @{ Paths = @($Paths); Readers = @{}; Partial = @{}; LastData = (Get-Date) }
}

function Read-FollowerLines {
    <#
        Returns whole lines, and updates LastData whenever any byte arrives:
        waitForNetwork emits bare printf(".") with no newline for up to 300s,
        which still counts as the board being alive.
    #>
    param($Follower)
    $lines = New-Object System.Collections.Generic.List[string]
    foreach ($p in $Follower.Paths) {
        if (-not $Follower.Readers.ContainsKey($p)) {
            if (-not (Test-Path -LiteralPath $p)) { continue }
            $fs = New-Object System.IO.FileStream($p, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::ReadWrite)
            $Follower.Readers[$p] = New-Object System.IO.StreamReader($fs, [System.Text.Encoding]::UTF8)
            $Follower.Partial[$p] = ''
        }
        $chunk = $Follower.Readers[$p].ReadToEnd()
        if ($chunk.Length -gt 0) {
            $Follower.LastData = Get-Date
            $buf = $Follower.Partial[$p] + $chunk
            # `\r*\n|\r` rather than `\r\n|\n|\r`, so the "\r\r\n" Python's
            # text-mode stdout produces is one break, not two.
            $hold = ''
            if ($buf.EndsWith("`r")) {
                $hold = "`r"
                $buf = $buf.Substring(0, $buf.Length - 1)
            }
            $parts = [regex]::Split($buf, "`r*`n|`r")
            $Follower.Partial[$p] = $parts[$parts.Length - 1] + $hold
            for ($i = 0; $i -lt $parts.Length - 1; $i++) { $lines.Add($parts[$i]) }
        }
    }
    return $lines
}

function Close-LogFollower {
    param($Follower)
    foreach ($r in $Follower.Readers.Values) { try { $r.Dispose() } catch { } }
}

function Test-Marker {
    param([string] $Line, [string] $Pattern)
    if ($Pattern.StartsWith('re:')) { return [bool]($Line -cmatch $Pattern.Substring(3)) }
    return $Line.Contains($Pattern)
}

function New-MatchState {
    param($Criteria, $Shared)

    $whitelist = @($Criteria.Banner) + @($Shared.Whitelist) + @($Criteria.Whitelist)
    $repeats = @(@($Shared.RepeatLimits) + @($Criteria.RepeatLimits) | Where-Object { $_ })

    $counts = @{}
    foreach ($r in $repeats) { $counts[$r.Pattern] = 0 }

    return @{
        Pending      = [System.Collections.Generic.List[string]]@($whitelist)
        Total        = $whitelist.Count
        Matched      = [System.Collections.Generic.List[string]]@()
        Blacklist    = @(@($Shared.Blacklist) + @($Criteria.Blacklist))
        Ignore       = @($Shared.IgnoreLines)
        FatalResets  = @($Shared.FatalResets)
        BenignResets = @($Criteria.BenignResets)
        AllowedResets = [int] $Criteria.AllowedResets
        Repeats      = $repeats
        Counts       = $counts
        ResetCount   = 0
        Context      = New-Object System.Collections.Generic.Queue[string]
        Verdict      = $null
        Reason       = ''
        FailLine     = ''
    }
}

function Update-MatchState {
    param($State, [string] $RawLine)

    $line = $AnsiPattern.Replace($RawLine, '')
    if ($line.Trim().Length -eq 0) { return }

    $State.Context.Enqueue($line)
    while ($State.Context.Count -gt 20) { [void] $State.Context.Dequeue() }

    # At DEBUG the driver dumps every AT exchange, and those lines carry the
    # modem's own literal "ERROR" text - never match them against a blacklist.
    foreach ($ig in $State.Ignore) {
        if (Test-Marker -Line $line -Pattern $ig) { return }
    }

    if ($State.Verdict) { return }

    foreach ($b in $State.Blacklist) {
        if (Test-Marker -Line $line -Pattern $b) {
            $State.Verdict = 'FAIL'
            $State.Reason = "blacklisted: $b"
            $State.FailLine = $line
            return
        }
    }

    if ($line -match 'rst:0x') {
        foreach ($f in $State.FatalResets) {
            if (Test-Marker -Line $line -Pattern $f) {
                $State.Verdict = 'FAIL'
                $State.Reason = 'reset by watchdog or brownout'
                $State.FailLine = $line
                return
            }
        }
        $benign = $false
        foreach ($b in $State.BenignResets) {
            if (Test-Marker -Line $line -Pattern $b) { $benign = $true; break }
        }
        if (-not $benign) {
            $State.ResetCount++
            if ($State.ResetCount -gt $State.AllowedResets) {
                $State.Verdict = 'FAIL'
                $State.Reason = "rebooted ($($State.ResetCount) resets seen; an esp_restart() failure path ran)"
                $State.FailLine = $line
                return
            }
        }
    }

    foreach ($r in $State.Repeats) {
        if ($r.ResetOn -and (Test-Marker -Line $line -Pattern $r.ResetOn)) {
            $State.Counts[$r.Pattern] = 0
        }
        if (Test-Marker -Line $line -Pattern $r.Pattern) {
            $State.Counts[$r.Pattern]++
            if ($State.Counts[$r.Pattern] -ge [int] $r.Max) {
                $State.Verdict = 'FAIL'
                $State.Reason = "stuck in a retry loop: '$($r.Pattern)' seen $($State.Counts[$r.Pattern]) times"
                $State.FailLine = $line
                return
            }
        }
    }

    for ($i = $State.Pending.Count - 1; $i -ge 0; $i--) {
        if (Test-Marker -Line $line -Pattern $State.Pending[$i]) {
            $State.Matched.Add($State.Pending[$i])
            $State.Pending.RemoveAt($i)
        }
    }

    if ($State.Pending.Count -eq 0) {
        $State.Verdict = 'PASS'
        $State.Reason = 'all whitelist markers seen'
    }
}

# ===========================================================================
# Replay mode - the only part of this that is testable without hardware
# ===========================================================================

function Invoke-Replay {
    param([string] $LogPath, [string] $ExampleName, $Criteria, $Shared)

    if (-not (Test-Path -LiteralPath $LogPath)) { throw "No such log: $LogPath" }
    $state = New-MatchState -Criteria $Criteria -Shared $Shared

    foreach ($line in [System.IO.File]::ReadAllLines($LogPath)) {
        Update-MatchState -State $state -RawLine $line
        if ($state.Verdict) { break }
    }
    if (-not $state.Verdict) {
        $state.Verdict = 'TIMEOUT'
        $state.Reason = 'log ended before every whitelist marker was seen'
    }

    Write-Head "Replay: $ExampleName"
    Write-Info "verdict : $($state.Verdict)"
    Write-Info "reason  : $($state.Reason)"
    Write-Info "matched : $($state.Matched.Count)/$($state.Total)"
    if ($state.FailLine) { Write-Bad "line    : $($state.FailLine)" }
    foreach ($p in $state.Pending) { Write-Warn "missing : $p" }
    return $state
}

# ===========================================================================
# One example, end to end
# ===========================================================================

function Invoke-Example {
    param(
        [string] $Name, $Criteria, $Shared,
        [string] $RunDir, [string] $EnvFile,
        [string] $SerialPort, [string] $ApnValue, [string] $Level
    )

    $result = [ordered]@{
        Example = $Name; Verdict = 'SKIPPED'; Reason = ''; Elapsed = 0
        Matched = 0; Total = 0; Missing = @(); FailLine = ''; Context = @()
        ApnPatch = @(); Infra = $Criteria.Infra; Attempts = 1
        # The log worth linking to, set to whichever phase the run reached.
        LogPath = ''
    }
    $runSw = [System.Diagnostics.Stopwatch]::StartNew()

    Write-Head "$Name"

    # -- stage -------------------------------------------------------------
    $projDir = Join-Path $RunDir $Name
    try {
        New-StagedExample -Name $Name -Dest $projDir
        Set-LocalComponentOverride -ManifestPath (Join-Path $projDir 'main\idf_component.yml') -ComponentRoot $RepoRoot
        Set-ExampleLogLevel -DefaultsPath (Join-Path $projDir 'sdkconfig.defaults') -Level $Level
        if ($ApnValue) {
            $result.ApnPatch = @(Set-ExampleApn -ProjectDir $projDir -ApnValue $ApnValue)
            if ($result.ApnPatch.Count -eq 0) {
                Write-Warn 'APN: no call site or define matched - this example will use the network-assigned APN.'
            } else {
                foreach ($p in $result.ApnPatch) { Write-Note "APN: patched $p" }
            }
        }
        Write-Note "staged in $projDir"
    } catch {
        $result.Verdict = 'STAGE_FAIL'
        $result.Reason = $_.Exception.Message
        Write-Bad $result.Reason
        return $result
    }

    if ($StageOnly) {
        $result.Verdict = 'STAGED'
        $result.Elapsed = [int] $runSw.Elapsed.TotalSeconds
        Write-Good 'staged (no build requested)'
        return $result
    }

    $buildLog = Join-Path $RunDir "$Name.build.log"
    $flashLog = Join-Path $RunDir "$Name.flash.log"
    $monLog = Join-Path $RunDir "$Name.monitor.log"
    $result.LogPath = $buildLog

    # -- open this example's window ----------------------------------------
    $win = $null
    try {
        $win = Start-ExampleWindow -RunDir $RunDir -EnvFile $EnvFile -Name $Name `
            -ProjDir $projDir -SerialPort $SerialPort -SkipFlash ([bool] $NoFlash)
        Write-Note "window opened (pid $($win.Id))"
    } catch {
        $result.Verdict = 'STAGE_FAIL'
        $result.Reason = $_.Exception.Message
        Write-Bad $result.Reason
        return $result
    }

    # -- build -------------------------------------------------------------
    try {
        $code = Wait-Phase -RunDir $RunDir -Name $Name -Phase 'build' -Proc $win `
            -TimeoutSeconds $BuildTimeoutSec -What 'building'
    } catch {
        $result.Verdict = 'BUILD_FAIL'
        $result.Reason = $_.Exception.Message
        Stop-Descendants -ParentId $win.Id
        Write-Bad $result.Reason
        return $result
    }
    if ($code -ne 0) {
        $result.Verdict = 'BUILD_FAIL'
        $result.Reason = "idf.py build exited $code"
        $result.Context = @(Get-BuildFailureContext -LogPath $buildLog)
        Write-Bad $result.Reason
        foreach ($c in ($result.Context | Select-Object -First 6)) { Write-Bad "  $c" }
        return $result
    }

    # The root CMakeLists prints one line per feature at configure time, and
    # ESP-IDF evaluates it twice - the first pass is early requirement
    # expansion, where every CONFIG_ is still empty and reads false - so look
    # for "true" anywhere. Seeing it proves the local checkout got built.
    #
    # Configure only runs when the build directory is new, so an incremental
    # build has no feature block at all. Check only when one is present, or
    # every retried build warns about output that was never going to be there.
    $buildText = Get-Content -LiteralPath $buildLog -Raw -ErrorAction SilentlyContinue
    if ($buildText -and $buildText -match 'WalterModem Configuration:') {
        foreach ($feat in @($Criteria.Features)) {
            if ($buildText -notmatch "$feat enabled: true") {
                Write-Warn "configure output never said '$feat enabled: true' - check the component resolved locally."
            }
        }
    }

    if ($NoFlash) {
        $result.Verdict = 'BUILT'
        $result.Elapsed = [int] $runSw.Elapsed.TotalSeconds
        Write-Good 'built (flash skipped)'
        return $result
    }

    # -- flash -------------------------------------------------------------
    $result.LogPath = $flashLog
    try {
        $code = Wait-Phase -RunDir $RunDir -Name $Name -Phase 'flash' -Proc $win `
            -TimeoutSeconds $FlashTimeoutSec -What 'flashing'
    } catch {
        $result.Verdict = 'FLASH_FAIL'
        $result.Reason = $_.Exception.Message
        Stop-Descendants -ParentId $win.Id
        Write-Bad $result.Reason
        return $result
    }
    if ($code -ne 0) {
        $result.Verdict = 'FLASH_FAIL'
        $result.Reason = "idf.py flash exited $code"
        $result.Context = @(Get-Content -LiteralPath $flashLog -Tail 25 -ErrorAction SilentlyContinue)
        Write-Bad $result.Reason
        return $result
    }
    $result.LogPath = $monLog

    # -- monitor and match -------------------------------------------------
    $budget = $TimeoutSec
    if (-not $StrictTimeout -and [int] $Criteria.TimeoutSec -gt 0) { $budget = [int] $Criteria.TimeoutSec }
    if ($NoTimeout) { $budget = 0 }

    $quiet = $QuietSec
    if (-not $StrictTimeout -and [int] $Criteria.QuietSec -gt 0) { $quiet = [int] $Criteria.QuietSec }

    # The window starts the monitor itself once the flash succeeds; from here
    # the controller only reads its log. One pass of this loop is one monitor
    # attempt - if the window offers a re-run, the matcher starts over.
    $follower = New-LogFollower -Paths @($monLog)
    $state = $null
    $attempt = 0

    try {
        while ($true) {
            $attempt++
            $state = New-MatchState -Criteria $Criteria -Shared $Shared
            $result.Total = $state.Total
            $sw = [System.Diagnostics.Stopwatch]::StartNew()
            $lastPrint = [DateTime]::MinValue
            # The follower is reused across attempts, so its idle clock still
            # holds the previous attempt's last line.
            $follower.LastData = Get-Date

            Write-Note ("watching for {0} markers; timeout {1}, quiet {2}s{3}" -f `
                    $state.Total, $(if ($budget) { "${budget}s" } else { 'none' }), $quiet,
                $(if ($attempt -gt 1) { "  (attempt $attempt)" } else { '' }))

            while ($true) {
                foreach ($line in (Read-FollowerLines -Follower $follower)) {
                    Update-MatchState -State $state -RawLine $line
                    if ($state.Verdict) { break }
                }
                if ($state.Verdict) { break }

                if ($win.HasExited) {
                    $state.Verdict = 'FAIL'; $state.Reason = "the $Name window was closed"
                    break
                }
                if ($budget -gt 0 -and $sw.Elapsed.TotalSeconds -gt $budget) {
                    $state.Verdict = 'TIMEOUT'
                    $state.Reason = "no verdict within ${budget}s"
                    break
                }
                $silent = ((Get-Date) - $follower.LastData).TotalSeconds
                if ($quiet -gt 0 -and $silent -gt $quiet) {
                    # Several examples return out of app_main on failure and
                    # then print nothing at all, forever.
                    $state.Verdict = 'SILENT'
                    $state.Reason = "no serial output for $([int]$silent)s"
                    break
                }

                if (((Get-Date) - $lastPrint).TotalSeconds -ge $StatusEverySec) {
                    $lastPrint = Get-Date
                    $next = if ($state.Pending.Count -gt 0) { $state.Pending[0] } else { '' }
                    if ($next.Length -gt 44) { $next = $next.Substring(0, 41) + '...' }
                    Write-Status ("{0,4}s  {1}/{2} matched  waiting for: {3}" -f `
                            [int]$sw.Elapsed.TotalSeconds, $state.Matched.Count, $state.Total, $next)
                }
                Start-Sleep -Milliseconds 200
            }

            Clear-Status

            # Tell the window how it went before stopping the monitor, so it
            # can decide whether to offer a re-run.
            Write-TextFile -Path (Join-Path $RunDir "$Name.verdict") `
                -Text "$($state.Verdict)|$($state.Reason)"

            # Kill only the monitor: the window survives with its scrollback,
            # and the serial port is freed for the next attempt or example.
            Stop-Descendants -ParentId $win.Id

            # Drain whatever landed between the verdict and the kill.
            Start-Sleep -Milliseconds 500
            foreach ($line in (Read-FollowerLines -Follower $follower)) {
                if (-not $state.Verdict) { Update-MatchState -State $state -RawLine $line }
            }

            if ($state.Verdict -eq 'PASS') { break }
            if ($NoRetry -or $CloseWindows) { break }
            if (-not (Wait-ForRetryDecision -RunDir $RunDir -Name $Name -Proc $win)) { break }

            Write-Note 'retrying: flashing and monitoring again'
            try {
                $code = Wait-Phase -RunDir $RunDir -Name $Name -Phase 'flash' -Proc $win `
                    -TimeoutSeconds $FlashTimeoutSec -What 'flashing'
            } catch {
                $state.Verdict = 'FLASH_FAIL'; $state.Reason = $_.Exception.Message
                break
            }
            if ($code -ne 0) {
                $state.Verdict = 'FLASH_FAIL'; $state.Reason = "idf.py flash exited $code"
                break
            }
        }
    } finally {
        Close-LogFollower -Follower $follower
    }

    $result.Attempts = $attempt
    $result.Verdict = $state.Verdict
    $result.Reason = $state.Reason
    $result.Matched = $state.Matched.Count
    $result.Missing = @($state.Pending)
    $result.FailLine = $state.FailLine
    $result.Context = @($state.Context.ToArray())
    $result.Elapsed = [int] $runSw.Elapsed.TotalSeconds

    if ($result.Attempts -gt 1) { Write-Note "$($result.Attempts) attempts" }
    if ($result.Verdict -eq 'PASS') {
        Write-Good "PASS in $($result.Elapsed)s"
    } else {
        Write-Bad "$($result.Verdict) after $($result.Elapsed)s - $($result.Reason)"
        if ($result.FailLine) { Write-Bad "  $($result.FailLine)" }
        foreach ($m in $result.Missing) { Write-Warn "  missing: $m" }
    }
    return $result
}

# ===========================================================================
# Report
# ===========================================================================

function Write-Report {
    param($Results, [string] $RunDir, $Settings)

    $bt = [string][char]96      # backtick, for markdown code spans
    $fence = $bt + $bt + $bt
    $code = { param($s) $bt + $s + $bt }

    $apnText = if ($Settings.Apn) { & $code $Settings.Apn } else { '_network-assigned_' }
    $overrideText = if ($Settings.Strict) { 'ignored' } else { 'applied' }
    $timeoutText = if ($Settings.NoTimeout) { 'none' } else { "$($Settings.TimeoutSec)s (per-example overrides $overrideText)" }

    $sb = New-Object System.Text.StringBuilder
    [void] $sb.AppendLine('# walter-esp-idf example test run')
    [void] $sb.AppendLine()
    [void] $sb.AppendLine("- when: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
    [void] $sb.AppendLine("- repo: $(& $code $RepoRoot) (branch $($Settings.Branch), HEAD $($Settings.Head))")
    [void] $sb.AppendLine("- esp-idf: $($Settings.IdfName) $(& $code $Settings.IdfPath)")
    [void] $sb.AppendLine("- port: $($Settings.Port)")
    [void] $sb.AppendLine("- apn: $apnText")
    [void] $sb.AppendLine("- log level: $($Settings.LogLevel)")
    [void] $sb.AppendLine("- timeout: $timeoutText, quiet watchdog $($Settings.QuietSec)s")
    [void] $sb.AppendLine("- work dir: $(& $code $RunDir)")
    [void] $sb.AppendLine()
    [void] $sb.AppendLine('| example | verdict | time | markers | attempts | reason |')
    [void] $sb.AppendLine('|---|---|---|---|---|---|')
    foreach ($r in $Results) {
        [void] $sb.AppendLine("| $($r.Example) | **$($r.Verdict)** | $($r.Elapsed)s | $($r.Matched)/$($r.Total) | $($r.Attempts) | $($r.Reason -replace '\|', '\|') |")
    }
    [void] $sb.AppendLine()

    # Only the failures get a section; a green run needs no prose.
    foreach ($r in $Results) {
        if ($r.Verdict -in @('PASS', 'STAGED', 'BUILT')) { continue }
        [void] $sb.AppendLine("## $($r.Example) - $($r.Verdict)")
        [void] $sb.AppendLine()
        [void] $sb.AppendLine("$($r.Reason)")
        [void] $sb.AppendLine()
        if ($r.FailLine) {
            [void] $sb.AppendLine('Offending line:')
            [void] $sb.AppendLine($fence)
            [void] $sb.AppendLine($r.FailLine)
            [void] $sb.AppendLine($fence)
        }
        if ($r.Missing.Count -gt 0) {
            [void] $sb.AppendLine('Whitelist markers never seen:')
            foreach ($m in $r.Missing) { [void] $sb.AppendLine("- $(& $code $m)") }
            [void] $sb.AppendLine()
        }
        # Only worth raising when the run reached the network; a build failure
        # has nothing to do with a dead public broker.
        if ($r.Infra -and $r.Verdict -in @('FAIL', 'TIMEOUT', 'SILENT')) {
            [void] $sb.AppendLine("Depends on external infrastructure: $($r.Infra)")
            [void] $sb.AppendLine()
        }
        if ($r.Context.Count -gt 0) {
            $label = if ($r.Verdict -eq 'BUILD_FAIL') { 'Build errors:' } else { 'Last lines:' }
            [void] $sb.AppendLine($label)
            [void] $sb.AppendLine($fence)
            foreach ($c in $r.Context) { [void] $sb.AppendLine($c) }
            [void] $sb.AppendLine($fence)
        }
        if ($r.LogPath) { [void] $sb.AppendLine("Full log: $(& $code $r.LogPath)") }
        [void] $sb.AppendLine()
    }

    $path = Join-Path $RunDir 'report.md'
    Write-TextFile -Path $path -Text $sb.ToString()
    return $path
}

# ===========================================================================
# Main
# ===========================================================================

if (-not (Test-Path -LiteralPath $CriteriaFile)) { throw "Missing criteria file: $CriteriaFile" }
$criteria = Import-PowerShellDataFile -LiteralPath $CriteriaFile
$shared = $criteria.Shared

Write-Host ''
Write-Host '  walter-esp-idf example tests' -ForegroundColor White
Write-Host "  repo: $RepoRoot" -ForegroundColor DarkGray

# -- replay mode: no IDF, no board ------------------------------------------
if ($ReplayLog) {
    if (-not $ReplayExample) { throw '-ReplayLog needs -ReplayExample.' }
    if (-not $criteria.Examples.ContainsKey($ReplayExample)) { throw "Unknown example '$ReplayExample'." }
    $st = Invoke-Replay -LogPath $ReplayLog -ExampleName $ReplayExample -Criteria $criteria.Examples[$ReplayExample] -Shared $shared
    exit $(if ($st.Verdict -eq 'PASS') { 0 } else { 1 })
}

# -- prompts ----------------------------------------------------------------
Write-Head 'Configuration'

$Saved = Get-SavedAnswers
if ($Saved.Count -gt 0) {
    Write-Note 'Defaults are what you chose last time - press Enter to keep each one (-Fresh ignores them).'
}

$install = $null
$envSpec = $null
if (-not $StageOnly) {
    $install = Select-IdfInstallation
    Write-Good "ESP-IDF $($install.Name) at $($install.IdfPath)"
    $envSpec = Resolve-IdfEnvironment -Install $install
    if (-not $envSpec.Python -or -not (Test-Path -LiteralPath $envSpec.Python)) {
        throw "No usable python found for $($install.Name). Re-run with -IdfPath, or repair the installation."
    }
}

$serialPort = ''
if (-not $StageOnly -and -not $NoFlash) {
    $serialPort = Select-SerialPort
    Write-Good "Board on $serialPort"
}

if (-not $Examples -or $Examples.Count -eq 0) {
    # Anything remembered that no longer exists is dropped, so a renamed
    # example cannot make Enter pick something invalid.
    $savedExamples = @(@(Get-SavedValue -Key 'Examples') | Where-Object { $SupportedExamples -contains $_ })
    if ($Yes -and $savedExamples.Count -gt 0) {
        $Examples = $savedExamples
    } else {
        $Examples = Read-MultiChoice -Title 'Which examples?' -Items $SupportedExamples -Default $savedExamples
    }
} elseif ($Examples.Count -eq 1 -and $Examples[0] -eq 'all') {
    $Examples = $SupportedExamples
}
foreach ($e in $Examples) {
    if ($SupportedExamples -notcontains $e) { throw "Unsupported example '$e'. Supported: $($SupportedExamples -join ', ')" }
}

if (-not $PSBoundParameters.ContainsKey('Apn')) {
    $savedApn = [string] (Get-SavedValue -Key 'Apn' -Fallback '')
    if ($Yes) {
        $Apn = $savedApn
    } else {
        Write-Host ''
        Write-Host '   Cellular APN. Leave empty to let the network assign one.' -ForegroundColor White
        $hint = if ($savedApn) { "[$savedApn]" } else { '[network-assigned]' }
        $ans = (Read-Host "   APN $hint").Trim()
        # Enter keeps last time's answer; "-" clears it back to unset.
        $Apn = if ($ans -eq '-') { '' } elseif ($ans) { $ans } else { $savedApn }
    }
}
if ($Apn -and $Apn -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]*$') {
    throw "APN '$Apn' contains characters that will not be injected into C source. Use letters, digits, dot, dash or underscore."
}

if (-not $LogLevel) {
    $levels = @('INFO', 'DEBUG', 'VERBOSE')
    $savedLevel = [string] (Get-SavedValue -Key 'LogLevel' -Fallback 'INFO')
    $levelIndex = [Math]::Max(0, $levels.IndexOf($savedLevel))
    if ($Yes) {
        $LogLevel = $levels[$levelIndex]
    } else {
        $LogLevel = Read-Choice -Title 'Log level?' -Items $levels -DefaultIndex $levelIndex -Label { param($x) $x }
    }
}
if ($LogLevel -ne 'INFO') {
    Write-Note 'DEBUG and above compile in the driver AT trace; expect a lot of output.'
}

if (-not $PSBoundParameters.ContainsKey('TimeoutSec') -and -not $NoTimeout) {
    $savedTimeout = [int] (Get-SavedValue -Key 'TimeoutSec' -Fallback $TimeoutSec)
    if ($savedTimeout -gt 0) { $TimeoutSec = $savedTimeout }
    if ([bool] (Get-SavedValue -Key 'NoTimeout' -Fallback $false)) { $NoTimeout = $true }

    if (-not $StageOnly -and -not $Yes) {
        Write-Host ''
        if (Read-YesNo 'Fail an example on a timeout?' (-not $NoTimeout)) {
            $NoTimeout = $false
            $ans = (Read-Host "   Timeout in seconds [$TimeoutSec]").Trim()
            if ($ans) {
                $n = 0
                if ([int]::TryParse($ans, [ref] $n) -and $n -gt 0) { $TimeoutSec = $n }
                else { Write-Warn "Not a number; keeping ${TimeoutSec}s." }
            }
        } else {
            $NoTimeout = $true
        }
    }
}

if (-not $WorkRoot) { $WorkRoot = Join-Path $env:TEMP 'walter-tests' }
$runId = Get-Date -Format 'yyyyMMdd-HHmmss'
$runDir = Join-Path $WorkRoot $runId
New-Item -ItemType Directory -Path $runDir -Force | Out-Null

# ESP-IDF nests object files deeply and CMake stops guaranteeing correctness
# past CMAKE_OBJECT_PATH_MAX (250); the project directory eats ~150 on its own.
if ($runDir.Length -gt 80) {
    Write-Warn "The work directory is $($runDir.Length) characters deep. ESP-IDF object paths may"
    Write-Warn "exceed CMAKE_OBJECT_PATH_MAX (250) and the build can fail oddly."
    Write-Warn 'Pass a shorter -WorkRoot, e.g. -WorkRoot C:\w.'
}

$branch = ''; $head = ''
try {
    $branch = (& git -C $RepoRoot rev-parse --abbrev-ref HEAD 2>$null)
    $head = (& git -C $RepoRoot rev-parse --short HEAD 2>$null)
} catch { }
$global:LASTEXITCODE = 0

# -- confirm ----------------------------------------------------------------
Write-Head 'Summary'
Write-Info "examples  : $($Examples -join ', ')"
if (-not $StageOnly) { Write-Info "esp-idf   : $($install.Name)  ($($install.IdfPath))" }
if ($serialPort) { Write-Info "port      : $serialPort" }
Write-Info "apn       : $(if ($Apn) { $Apn } else { '(network-assigned)' })"
Write-Info "log level : $LogLevel"
Write-Info "timeout   : $(if ($NoTimeout) { 'none' } else { "${TimeoutSec}s" })   quiet: ${QuietSec}s"
Write-Info "work dir  : $runDir"
Write-Note "the repository at $RepoRoot is only read, never written"

if (-not $Yes) {
    Write-Host ''
    if (-not (Read-YesNo 'Start?' $true)) { Write-Note 'Nothing done.'; exit 2 }
}

# Remembered only once the run is actually going ahead, so answering the
# prompts and then backing out does not overwrite what worked last time.
Save-Answers -Answers @{
    IdfVersion = $(if ($install) { $install.Name } else { Get-SavedValue -Key 'IdfVersion' })
    Port       = $(if ($serialPort) { $serialPort } else { Get-SavedValue -Key 'Port' })
    Examples   = @($Examples)
    Apn        = $Apn
    LogLevel   = $LogLevel
    TimeoutSec = $TimeoutSec
    NoTimeout  = [bool] $NoTimeout
}

# -- run --------------------------------------------------------------------
$envFile = ''
if (-not $StageOnly) {
    $envFile = Join-Path $runDir 'idfenv.json'
    Write-TextFile -Path $envFile -Text ($envSpec | ConvertTo-Json -Depth 5)
}

$results = @()
foreach ($name in $Examples) {
    $results += Invoke-Example -Name $name -Criteria $criteria.Examples[$name] -Shared $shared `
        -RunDir $runDir -EnvFile $envFile `
        -SerialPort $serialPort -ApnValue $Apn -Level $LogLevel
}

# -- report -----------------------------------------------------------------
$settings = @{
    Branch = $branch; Head = $head; Port = $serialPort; Apn = $Apn; LogLevel = $LogLevel
    TimeoutSec = $TimeoutSec; NoTimeout = [bool] $NoTimeout; Strict = [bool] $StrictTimeout; QuietSec = $QuietSec
    IdfName = $(if ($install) { $install.Name } else { 'n/a' })
    IdfPath = $(if ($install) { $install.IdfPath } else { 'n/a' })
}
$reportPath = Write-Report -Results $results -RunDir $runDir -Settings $settings

Write-Head 'Result'
foreach ($r in $results) {
    $colour = switch ($r.Verdict) {
        'PASS' { 'Green' }
        'STAGED' { 'Green' }
        'BUILT' { 'Green' }
        default { 'Red' }
    }
    Write-Host ("   {0,-12} {1,-11} {2,5}s  {3}/{4}  {5}" -f $r.Example, $r.Verdict, $r.Elapsed, $r.Matched, $r.Total, $r.Reason) -ForegroundColor $colour
}
Write-Host ''
Write-Info "report: $reportPath"
Write-Info "logs  : $runDir"

$bad = @($results | Where-Object { $_.Verdict -notin @('PASS', 'STAGED', 'BUILT') })
exit $(if ($bad.Count -gt 0) { 1 } else { 0 })
