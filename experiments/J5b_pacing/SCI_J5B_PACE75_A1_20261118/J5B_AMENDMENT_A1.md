# J5b protocol amendment A1 — reading the BSSID (18 Nov 2026, before any counted replay)

Applies to `J5B_PROTOCOL_FREEZE.md` (package `SCI_J5B_PACE75_20261118`, freeze file c14886b7…). Declared after the first
J5b attempt stopped in the pilots. No counted replay had run.

## What happened
- First attempt, results folder `J5B_MAIN`, 18 Nov 2026, about 13:48–13:55 UTC.
- Pilots S2–S6 all passed (same values as in J5), but `all_pilots_valid` was false, so the run halted as frozen.
- The pilot `PIL_N_Q2_B0` was marked invalid with reason `bssid`. However:
  - 22 of its 23 BSSID samples, and the samples taken before and after the replay, show the frozen BSSID
    5c:e9:31:85:b4:1b;
  - **one sample**, 18.6 s after the trigger (mode N, kernel queue full), came back **empty**: no BSSID, no frequency, no
    signal;
  - the edge received all 257 messages, and the clock offset before and after was below 0.05 ms;
  - the kernel log and the system log show no disconnect, deauthentication or wpa_supplicant event in 13:40–13:56 UTC
    (only the expected `sudo tc` and `sudo iw … power_save off` lines).
- Cause: the frozen J5 reader returns "BSSID = none" whenever one call of `iw dev wlan0 link` fails (an error or the 5 s
  timeout). The validity rule then counts this as a BSSID change. The sample count (23 in a 35 s replay, at 1 Hz)
  suggests that some `iw` calls were slow in this replay. A failed read is not a roam or a disconnect.

## Change (one file added, one entry point added, the scripts adjusted; the frozen files are not edited)
- `j5b_sysops_a1.py`: the BSSID reader separates two cases.
  - `Not connected.` from `iw` is a **real disconnect**. It is returned at once as "BSSID = none", so the replay stays
    invalid, as before.
  - A **failed read** (an error, a timeout, or output with neither line) is retried up to 2 more times, 0.3 s apart,
    with a 2 s timeout each. If all 3 attempts fail, "BSSID = none" is returned and the replay is invalid, as in J5.
  - Every sample records its attempts and, if it failed, the reason. All retries are reported.
- **The validity rule is unchanged:** a replay is invalid if any sample shows another BSSID or none.
- `j5b_robot_a1.py`: the J5b entry point with this reader. `j5b_robot.py` is kept unchanged.
- `JALANKAN_J5B_A1_EDGE.sh` / `JALANKAN_J5B_A1_ROBOT.sh` differ from the J5b scripts in four ways:
  - they write to a **new results folder `J5B_MAIN_A1`**;
  - they run the A1 entry point;
  - they run the A1 tests;
  - the Pi script reads a halt from the state file. In the first attempt a halt was hidden by the known abort on exit
    and reported as "stopped, run again".
- `freeze_j5b_a1.py` / `J5B_A1_FREEZE_HASHES.json` freeze the amended package. The A1 tests check that every file of
  the original J5b freeze is unchanged.

## What stays as frozen
Everything else in J5b: the pace 0.75 c, matrix, pilots and their limits (the pilots are run again from the start in
`J5B_MAIN_A1`), the background pilot, the predictions, the tests K1–K3 and R1–R2, the verdict, and the vendored J5
package.

## Reporting
- The first attempt (`J5B_MAIN`, pilots only, halted) is kept on both hosts. It is reported in the ESM with this
  amendment.
- None of its replays is counted.
