# Tests

Builds, flashes and runs the `examples/` projects on a real Walter board and
reports PASS or FAIL for each, decided from the serial log. Run it before
opening a PR to check you have not broken anything.

There are no unit tests here: this component is a driver for a cellular modem
over UART, so "does it work" means an example talking to a live network.

## Prerequisites

- **Windows**, with the Windows PowerShell that ships in the Start menu.
- **ESP-IDF**, installed by the Espressif Installation Manager or the offline
  installer. The script finds the installed versions; you pick one.
- **A Walter board on USB**, with an antenna and an activated SIM. There is
  nothing to run without one.
- **Reachable demo servers.** Every example except `bluecherry` talks to public
  infrastructure - `coap.me`, `quickspot.io`, `broker.emqx.io`,
  `walterdemo.quickspot.io`. `bluecherry` needs a BlueCherry tenant, and
  `positioning` needs a GNSS antenna with a view of the sky.

## Running

```powershell
cd tests
.\Run.ps1
```

It asks for the ESP-IDF version, the board, which examples to run, the APN, the
log level and the timeout. **Your answers are remembered** and become the
defaults next time, so a repeat run is Enter, Enter, Enter. At the APN prompt,
`-` clears it back to network-assigned.

`-Fresh` ignores what was remembered. It lives in
`%LOCALAPPDATA%\walter-esp-idf\tests-last-run.json`, outside the repository, so
it never shows up in `git status`; delete it to start over.

Every prompt also has a parameter, so a run can skip them entirely:

```powershell
.\Run.ps1 -IdfVersion v5.5.2 -Port COM3 -Examples tcp,udp -Apn '' -Yes
```

`-Yes` takes the remembered answer for anything you do not pass, which makes a
repeat run unattended. `Get-Help .\Run.ps1 -Detailed` lists every parameter.

| File | Role |
|---|---|
| `Run.ps1` | the entry point - the only one you run |
| `Criteria.psd1` | pass/fail criteria per example; edit this, not the script |
| `internal\Worker.ps1` | runs one example in its own window; launched for you |

## What a run does

Each example is copied to `%TEMP%\walter-tests\<timestamp>\`, repointed at this
checkout, patched with your APN and log level, then built, flashed and monitored
in **its own console window**. The repository is never written to - check with
`git status` afterwards.

Those windows are left open when the example finishes, so the compiler output,
esptool and the serial log stay together in one scrollback you can read. Close
them yourself, or pass `-CloseWindows`. Serial lines are coloured by log level
the way ESP-IDF does it.

The same output is written to `<example>.{build,flash,monitor}.log` beside a
`report.md`, so nothing is lost once the windows are closed. The exit code is
non-zero if anything did not pass.

## Verdicts

| Verdict | Meaning |
|---|---|
| `PASS` | every whitelist marker appeared |
| `FAIL` | a blacklist marker appeared, the board rebooted unexpectedly, or a retry loop hit its limit |
| `TIMEOUT` | the budget ran out with markers still outstanding |
| `SILENT` | no serial output at all for `-QuietSec`. Several examples return out of `app_main` on failure and then print nothing, forever |
| `BUILD_FAIL` / `FLASH_FAIL` | `idf.py` exited non-zero |
| `STAGED` / `BUILT` | `-StageOnly` / `-NoFlash` ran to completion |

A red run is not always a regression. `report.md` prints the external
infrastructure each failing example depends on, because a dead public broker
looks exactly like a bug in the driver.

## When something fails, you get a retry

A failing phase does not fail the example outright. Its window asks what to do,
and waits as long as you like:

| What failed | Enter | Close the window |
|---|---|---|
| build | build again | give up, record `BUILD_FAIL` |
| flash | flash again, **without** rebuilding | give up, record `FLASH_FAIL` |
| the test itself | flash and monitor again, resetting the board | give up, record the verdict |

The third one earns its keep: a `FAIL` because a demo server was unreachable
costs a re-run rather than a three-minute rebuild. `-NoRetry` turns this off and
fails on the first problem, which is what an unattended run wants.

## Tuning the criteria

`Criteria.psd1` holds the banner, whitelist, blacklist, retry caps, allowed
resets and timeouts per example; the comments in it explain each field. Markers
are either `'literal text'` (case-sensitive substring) or `'re:<regex>'`.

Two flags check an edit without touching hardware:

```powershell
.\Run.ps1 -ReplayLog $env:TEMP\walter-tests\20260922-093014\tcp.monitor.log -ReplayExample tcp
.\Run.ps1 -StageOnly
```

`-ReplayLog` runs a saved monitor log back through the matcher and prints the
verdict it would have given, which markers matched and which were outstanding -
usually enough to see why a criterion was too strict or too loose. `-StageOnly`
stages and patches the examples and stops, so you can read a staged
`main/idf_component.yml` or confirm `git status` is clean.

## Notes

- **The APN only reaches `bluecherry`.** The other examples call
  `definePDPContext()` with no arguments, and nothing can reach that from
  outside. The harness patches both shapes it can find and prints which one
  fired; leaving the APN empty and letting the network assign one is usually
  what you want.
- **`modem_firmware_flash` is excluded on purpose**: it does not use this
  component, it reflashes the modem, and a run takes up to 30 minutes.
- One ESP-IDF version and one board per run.
