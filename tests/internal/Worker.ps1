<#
.SYNOPSIS
    Internal helper for ..\Run.ps1 - not an entry point. Runs one example's
    build, flash and monitor in its own window.

.DESCRIPTION
    ..\Run.ps1 opens one of these per example and leaves it open, so the whole
    story for that example - compiler output, esptool, serial log - stays in
    one scrollback. It has to be a real console window: esp_idf_monitor exits
    with "Monitor requires standard input to be attached to TTY" otherwise.

    A failing phase offers a retry here instead of failing the example
    outright. Closing the window is the way to give up; the controller notices
    and records the failure.

    Files written into -RunDir, which Run.ps1 watches:
        <name>.started       this window is up and the IDF env is set
        <name>.build.done    exit code of idf.py build, once settled
        <name>.flash.done    exit code of idf.py flash, once settled
        <name>.monitor.done  exit code of idf.py monitor (after it is killed)
        <name>.waiting       a person is being asked whether to retry, so the
                             controller should hold its phase timeout
        <name>.retry         a retry was asked for; reset the matcher
        <name>.<phase>.log   everything that phase printed

    And read from -RunDir:
        <name>.verdict       PASS, or <VERDICT>|<reason>, written by the
                             controller once it has judged the serial log
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string] $RunDir,
    [Parameter(Mandatory = $true)][string] $EnvFile,
    [Parameter(Mandatory = $true)][string] $Name,
    [Parameter(Mandatory = $true)][string] $ProjDir,
    [string] $Port,
    [switch] $NoFlash,
    [switch] $NoRetry,
    [string] $IdfTarget = 'esp32s3'
)

$ErrorActionPreference = 'Continue'

try { $Host.UI.RawUI.WindowTitle = "Walter tests - $Name" } catch { }

function Write-Banner {
    param([string] $Text, [string] $Colour = 'Cyan')
    Write-Host ''
    Write-Host ('=' * 78) -ForegroundColor $Colour
    Write-Host "  $Text" -ForegroundColor $Colour
    Write-Host ('=' * 78) -ForegroundColor $Colour
}

# ---------------------------------------------------------------------------
# ESP-IDF environment
# ---------------------------------------------------------------------------

$envSpec = Get-Content -LiteralPath $EnvFile -Raw | ConvertFrom-Json

if ($envSpec.SourceProfile) {
    # Fallback: the controller could not parse the profile's -e output, so
    # dot-source it. That also runs the profile's trailing `eim select`, which
    # rewrites idfSelectedId in eim_idf.json.
    Write-Banner "Activating ESP-IDF by sourcing $($envSpec.SourceProfile)" 'Yellow'
    . $envSpec.SourceProfile
} else {
    Write-Banner "$Name - ESP-IDF $($envSpec.Name) ($($envSpec.IdfPath))"
    foreach ($p in $envSpec.Vars.PSObject.Properties) {
        Set-Item -Path ("env:" + $p.Name) -Value $p.Value
    }
    if ($envSpec.PathPrepend) { $env:PATH = "$($envSpec.PathPrepend);$($env:PATH)" }
}

# Python block-buffers stdout in 8KB chunks when it is a pipe rather than a
# console, which would hand the matcher the serial log in bursts, minutes late.
$env:PYTHONUNBUFFERED = '1'

$python = $envSpec.Python
$idfPy = $envSpec.IdfPy
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)

# Colour goes to the screen, never to the log file: the file is what the
# matcher reads and what you open in an editor afterwards.
$AnsiPattern = [regex]("$([char]27)\[[0-9;?]*[ -/]*[@-~]")

# Levels are coloured here rather than left to the monitor, whose own
# auto-colouring is anchored at the start of the line (`^(I|W|E) \(`) - so the
# --timestamps prefix defeats it - and never covers D or V.
$LevelPattern = [regex]'^(?:\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:\.\d+)? )?[^A-Za-z]*([EWIDV]) \(\d+\)'
$LevelColour = @{
    'E' = [ConsoleColor]::Red
    'W' = [ConsoleColor]::Yellow
    'I' = [ConsoleColor]::Green
    'D' = [ConsoleColor]::DarkGray
    'V' = [ConsoleColor]::DarkGray
}

Set-Content -LiteralPath (Join-Path $RunDir "$Name.started") -Value $PID -Encoding ascii

# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------

function ConvertTo-Arg {
    param([string] $Value)
    if ($Value -match '[\s"]') { return '"' + ($Value -replace '"', '\"') + '"' }
    return $Value
}

function Invoke-Phase {
    <#
        Runs one idf.py phase, echoing every line to this window and to the
        phase's log file.

        Two details are load-bearing. Parts of the toolchain write "\r\n"
        through Python's text-mode stdout, which translates the "\n" again and
        emits "\r\r\n", so anything treating a lone CR as a terminator sees an
        extra blank line between every log line; splitting on `\r*\n|\r`
        collapses that to one break while keeping genuinely blank lines.

        And going through cmd's `2>&1` merges stderr in the right order with
        one redirected handle rather than two (reading two synchronously can
        deadlock), and avoids the RemoteException lines PowerShell injects
        whenever a native command writes to stderr.
    #>
    param([string] $Phase, [string[]] $Arguments, [switch] $Append)

    $log = Join-Path $RunDir "$Name.$Phase.log"
    Write-Banner "$Name - $Phase"
    Write-Host "cwd: $ProjDir" -ForegroundColor DarkGray
    Write-Host "run: idf.py $($Arguments -join ' ')" -ForegroundColor DarkGray
    Write-Host ''

    $sw = $null
    $proc = $null
    $exitCode = 1
    try {
        # Retries append rather than truncate: the controller is tailing this
        # file, and truncating under it leaves its read position past the end.
        $sw = New-Object System.IO.StreamWriter($log, $Append.IsPresent, $utf8NoBom)
        $sw.AutoFlush = $true   # the controller tails this live
        if ($Append) { $sw.WriteLine("--- $Phase retry ---") }

        $cmdLine = (@($python, $idfPy) + $Arguments | ForEach-Object { ConvertTo-Arg $_ }) -join ' '

        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = $env:ComSpec
        # /s plus one outer pair of quotes: without it cmd strips the first
        # and last quote of the command line, so any quoted path - which is
        # any path containing a space - comes apart.
        $psi.Arguments = "/s /c `"$cmdLine 2>&1`""
        $psi.WorkingDirectory = $ProjDir
        $psi.UseShellExecute = $false
        $psi.RedirectStandardOutput = $true
        # stdin is deliberately NOT redirected: esp_idf_monitor exits unless
        # it sees a TTY there.

        $proc = New-Object System.Diagnostics.Process
        $proc.StartInfo = $psi
        [void] $proc.Start()

        # ANSI is stripped for both destinations: the console is coloured from
        # the log level instead, and the log file stays plain text.
        $emit = {
            param([string] $Text)
            $clean = $AnsiPattern.Replace($Text, '')
            $m = $LevelPattern.Match($clean)
            if ($m.Success -and $LevelColour.ContainsKey($m.Groups[1].Value)) {
                $previous = [Console]::ForegroundColor
                try {
                    [Console]::ForegroundColor = $LevelColour[$m.Groups[1].Value]
                    [Console]::Out.WriteLine($clean)
                } finally {
                    [Console]::ForegroundColor = $previous
                }
            } else {
                [Console]::Out.WriteLine($clean)
            }
            $sw.WriteLine($clean)
        }

        $buf = New-Object char[] 8192
        $partial = ''
        while (($n = $proc.StandardOutput.Read($buf, 0, $buf.Length)) -gt 0) {
            $chunk = $partial + [string]::new($buf, 0, $n)
            # Hold a trailing CR back, so a "\r" and "\n" split across two
            # reads are not counted as two line breaks.
            $hold = ''
            if ($chunk.EndsWith("`r")) {
                $hold = "`r"
                $chunk = $chunk.Substring(0, $chunk.Length - 1)
            }
            $parts = [regex]::Split($chunk, "`r*`n|`r")
            for ($i = 0; $i -lt $parts.Length - 1; $i++) { & $emit $parts[$i] }
            $partial = $parts[$parts.Length - 1] + $hold
        }
        if ($partial.Length -gt 0) { & $emit $partial }

        $proc.WaitForExit()
        $exitCode = $proc.ExitCode
    } catch {
        $msg = "[worker] $Phase failed to start: $($_.Exception.Message)"
        Write-Host $msg -ForegroundColor Red
        if ($sw) { $sw.WriteLine($msg) }
        $exitCode = 1
    } finally {
        if ($sw) { $sw.Dispose() }
        if ($proc) { $proc.Dispose() }
    }

    return $exitCode
}

# ---------------------------------------------------------------------------
# Retry prompts
#
# Closing the window is the give-up gesture, so Request-Retry either returns
# $true or never returns because the process is gone. The waiting marker tells
# the controller to hold its phase timeout while someone is deciding.
# ---------------------------------------------------------------------------

function Request-Retry {
    param([string] $Headline, [string] $Action)

    if ($NoRetry) { return $false }

    $waiting = Join-Path $RunDir "$Name.waiting"
    Write-Host ''
    Write-Host ('-' * 78) -ForegroundColor Yellow
    Write-Host "  $Headline" -ForegroundColor Yellow
    Write-Host "  Press Enter to $Action, or close this window to give up and" -ForegroundColor Yellow
    Write-Host '  record the failure.' -ForegroundColor Yellow
    Write-Host ('-' * 78) -ForegroundColor Yellow
    try {
        Set-Content -LiteralPath $waiting -Value $Action -Encoding ascii
        [void] (Read-Host '  ')
    } finally {
        Remove-Item -LiteralPath $waiting -Force -ErrorAction SilentlyContinue
    }
    return $true
}

function Invoke-PhaseUntilSettled {
    <#
        Runs a phase, offering a retry each time it fails. The .done file is
        written only once the phase has settled, so the controller never sees
        a failing exit code someone is still deciding about - it keeps waiting.

        Returns $false only under -NoRetry; otherwise it either succeeds or the
        window is closed and this process ends here.
    #>
    param([string] $Phase, [string[]] $Arguments, [string] $Headline, [string] $Action)

    $done = Join-Path $RunDir "$Name.$Phase.done"
    $attempt = 0
    while ($true) {
        $attempt++
        if (Test-Path -LiteralPath $done) { Remove-Item -LiteralPath $done -Force }
        # Retries append. Truncating would throw away the failure someone is
        # about to read, and a rebuild does not re-run CMake configure, so the
        # controller's feature check would lose the only copy of that output.
        $code = Invoke-Phase -Phase $Phase -Arguments $Arguments -Append:($attempt -gt 1)
        if ($code -eq 0) {
            Set-Content -LiteralPath $done -Value $code -Encoding ascii
            return $true
        }
        Write-Host ''
        Write-Host "$Name - $Phase failed (exit $code)." -ForegroundColor Red
        if (-not (Request-Retry -Headline $Headline -Action $Action)) {
            Set-Content -LiteralPath $done -Value $code -Encoding ascii
            return $false
        }
    }
}

# The controller writes the verdict once it has judged the serial log, so the
# window can tell a pass from a failure worth re-running.
function Wait-ForVerdict {
    $verdictFile = Join-Path $RunDir "$Name.verdict"
    for ($i = 0; $i -lt 120; $i++) {
        if (Test-Path -LiteralPath $verdictFile) {
            return (Get-Content -LiteralPath $verdictFile -Raw).Trim()
        }
        Start-Sleep -Milliseconds 250
    }
    return 'UNKNOWN|the controller did not report a verdict'
}

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

if (Invoke-PhaseUntilSettled -Phase 'build' -Arguments @("-DIDF_TARGET=$IdfTarget", 'build') `
        -Headline 'The build failed.' -Action 'build again') {

    if (-not $NoFlash -and $Port) {
        $attempt = 0
        while ($true) {
            $attempt++
            if (-not (Invoke-PhaseUntilSettled -Phase 'flash' -Arguments @('-p', $Port, 'flash') `
                        -Headline 'Flashing failed.' -Action 'flash again without rebuilding')) { break }

            # Monitor resets the target on start, so the boot banner is
            # captured. It runs until the controller kills it, which is why its
            # exit code is not a failure signal.
            $monitorDone = Join-Path $RunDir "$Name.monitor.done"
            if (Test-Path -LiteralPath $monitorDone) { Remove-Item -LiteralPath $monitorDone -Force }
            $code = Invoke-Phase -Phase 'monitor' -Arguments @('-p', $Port, 'monitor', '--timestamps') `
                -Append:($attempt -gt 1)
            Set-Content -LiteralPath $monitorDone -Value $code -Encoding ascii

            $verdict = Wait-ForVerdict
            if ($verdict -like 'PASS*') {
                Write-Host ''
                Write-Host "$Name - PASS" -ForegroundColor Green
                break
            }

            $reason = if ($verdict -match '\|(.*)$') { $Matches[1] } else { $verdict }
            Write-Host ''
            Write-Host "$Name - did not pass: $reason" -ForegroundColor Red
            if (-not (Request-Retry -Headline 'The test did not pass.' `
                        -Action 'flash and monitor again, resetting the board')) { break }

            # Clear the previous attempt's markers *before* announcing the
            # retry: the controller waits for flash.done as soon as it sees
            # .retry, and a leftover one would let it start matching while the
            # board is still being flashed.
            foreach ($stale in @('flash.done', 'monitor.done')) {
                $p = Join-Path $RunDir "$Name.$stale"
                if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath $p -Force }
            }

            # Tells the controller to reset the matcher for a new attempt.
            Set-Content -LiteralPath (Join-Path $RunDir "$Name.retry") -Value $attempt -Encoding ascii
        }
    }
}

Write-Banner "$Name - finished. Scroll back for the full output; close this window when done." 'Green'
