#!/usr/bin/env python3
"""Check a RubbleSense scan log (JSON lines) against the rules the project promises.

    python3 tools/validate_scan_log.py scans.jsonl [more files...]      # '-' = stdin
    python3 tools/validate_scan_log.py capture.txt                      # a raw console capture works too

Reads every line that starts with '{' (console chatter and '#' lines are skipped), e.g. the output of the
`log` console command, the file send_iq_replay.py --capture writes, or a copy of the serial terminal. For
every scan record it verifies that

  * the record is complete and its fields are consistent with each other (verdict <-> evidence, the line
    fields of a candidate against the radar geometry, segment counts, link statistics, baseline <-> sensitivity)
  * NO_RELIABLE_PATTERN carries the mandatory sentence verbatim and no candidate fields
  * a negative never stands at a raised noise floor (that is LOW_SENSITIVITY) and the sensitivity statement
    matches the numbers
  * no text claims an area is clear / safe / empty, names a percentage, or says nobody is there
  * nothing outside the known schema appears (a new field must be added here on purpose, with its rule)

It is a consistency check of what the device wrote, not a check that the verdict was right: only recordings
of real rubble with known ground truth can say that. Exit status 0 = no violations (warnings are listed).
"""
import argparse
import json
import re
import sys

VERDICTS = {"INVALID_SCAN", "CONFOUNDED", "LOW_SENSITIVITY", "NO_RELIABLE_PATTERN", "POSSIBLE_PATTERN", "CANDIDATE_PATTERN"}
MANDATORY = "No reliable life-sign pattern detected in this scan. DO NOT clear area."
SAFETY_NOTICE = "Aid only. Never infer absence of survivor from a negative scan."
REASONS = {"none", "too_short", "sensor_motion", "radar_dropout", "imu_missing", "config", "no_signal",
           "adc_saturated", "radar_timing"}
EVIDENCE = {"strong", "moderate", "weak", "none"}
SOURCES = {"SIM", "SERIAL"}
IMUS = {"hw", "sim", "host"}
BASELINES = {"none", "calibrated", "sim_default"}
SENS = {"unknown", "ok", "low"}
MAX_BINS = 24

TOP_KEYS = {"v", "fw", "scan_id", "uptime_s", "duration_s", "source", "preset", "imu", "verdict", "evidence", "result",
            "operator_action", "invalid_reason", "bin", "range_m", "hz", "bpm", "snr_db", "margin_db", "segments_agree",
            "vetted", "segments_total", "segments_valid", "segments_invalid", "floor_db", "sensitivity", "baseline_src",
            "baseline_db", "floor_delta_db", "cpe_applied", "vibration", "vibration_hz", "platform_motion_mm_rms",
            "radar", "link", "segments", "params", "safety_notice"}
REQUIRED = TOP_KEYS - {"bin", "range_m", "hz", "bpm", "snr_db", "margin_db", "segments_agree", "vetted", "baseline_db",
                       "floor_delta_db"}
LINE_KEYS = {"bin", "range_m", "hz", "bpm", "snr_db", "margin_db", "segments_agree", "vetted"}
RADAR_KEYS = {"name", "fs_hz", "range_start_m", "range_step_m", "wavelength_mm", "bins"}
LINK_KEYS = {"lost_frames", "crc_errors", "timing_flagged", "timestamps"}
SEG_KEYS = {"valid", "reason", "det", "platform", "untested", "confirmed", "best_bin", "best_hz", "best_margin_db",
            "floor_db", "bad_frac", "cpe", "vib"}
PARAM_KEYS = {"pfa", "pfc", "band_hz", "min_bin", "cpe", "guard", "low_sens_db", "require_imu"}

# phrases that are allowed to contain words the claim detector would otherwise flag
ALLOWED_PHRASES = [r"do not clear area", r"never infer absence of survivor", r"a negative scan never means",
                   r"not (a |an )?(proof|clearance)", r"\bnot confirmed\b", r"\bunconfirmed\b"]
DANGEROUS = [
    (r"\b(area|zone|site|room|void|space)\s+(is\s+)?(clear|safe|empty|free)\b", "claims the area is clear / safe / empty"),
    (r"\ball[ -]clear\b", "says all clear"),
    (r"\bsafe to\b", "says it is safe to do something"),
    (r"\bno\s+(survivors?|victims?|people|persons?|life|one)\b", "says nobody / no survivor is there"),
    (r"\bnobody\b", "says nobody is there"),
    (r"\b(confirmed|certain|guaranteed)\b", "claims certainty"),
    (r"\d\s*%", "names a percentage (no calibrated probability exists)"),
    (r"\b(confidence|probability|likelihood)\b", "names a confidence / probability"),
]


class Checker:
    def __init__(self):
        self.violations = []
        self.warnings = []
        self.cur = ""

    def bad(self, msg):
        self.violations.append("%s: %s" % (self.cur, msg))

    def warn(self, msg):
        self.warnings.append("%s: %s" % (self.cur, msg))

    def num(self, d, key, lo=None, hi=None, integer=False):
        v = d.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            self.bad("'%s' is not a number (%r)" % (key, v))
            return None
        if integer and (v != int(v)):
            self.bad("'%s' must be an integer (%r)" % (key, v))
        if (lo is not None and v < lo) or (hi is not None and v > hi):
            self.bad("'%s' = %r is outside [%s, %s]" % (key, v, lo, hi))
        return v

    def boolean(self, d, key):
        if not isinstance(d.get(key), bool):
            self.bad("'%s' is not a boolean (%r)" % (key, d.get(key)))
            return None
        return d[key]

    def keys(self, d, allowed, required, where):
        extra = set(d) - allowed
        if extra:
            self.bad("unknown field(s) in %s: %s" % (where, ", ".join(sorted(extra))))
        missing = required - set(d)
        if missing:
            self.bad("missing field(s) in %s: %s" % (where, ", ".join(sorted(missing))))

    def text_claims(self, text, field):
        low = text.lower()
        for ph in ALLOWED_PHRASES:
            low = re.sub(ph, " ", low)
        for rx, why in DANGEROUS:
            m = re.search(rx, low)
            if m:
                self.bad("text of '%s' %s: \"%s\"" % (field, why, m.group(0)))

    def record(self, r):
        self.keys(r, TOP_KEYS, REQUIRED, "the record")
        if r.get("v") != 1:
            self.bad("schema version %r (this checker knows version 1)" % (r.get("v"),))
        sid = r.get("scan_id", "")
        if not isinstance(sid, str) or not re.fullmatch(r"RS-\d{4,}", sid):
            self.bad("scan_id %r is not RS-nnnn" % (sid,))
        verdict = r.get("verdict")
        if verdict not in VERDICTS:
            self.bad("verdict %r is not one of the defined verdicts (there is deliberately no 'clear' verdict)" % (verdict,))
            return
        for key, allowed in (("evidence", EVIDENCE), ("invalid_reason", REASONS), ("source", SOURCES), ("imu", IMUS),
                             ("baseline_src", BASELINES), ("sensitivity", SENS)):
            if r.get(key) not in allowed:
                self.bad("%s %r is not one of %s" % (key, r.get(key), sorted(allowed)))
        if r.get("safety_notice") != SAFETY_NOTICE:
            self.bad("safety_notice is missing or altered")

        # ---- texts
        for field in ("result", "operator_action"):
            if not isinstance(r.get(field), str) or not r.get(field):
                self.bad("'%s' is empty" % field)
            else:
                self.text_claims(r[field], field)
        if verdict == "NO_RELIABLE_PATTERN" and r.get("result") != MANDATORY:
            self.bad("a negative scan must carry the mandatory sentence verbatim, got %r" % (r.get("result"),))

        # ---- sub-objects
        radar = r.get("radar") if isinstance(r.get("radar"), dict) else {}
        link = r.get("link") if isinstance(r.get("link"), dict) else {}
        params = r.get("params") if isinstance(r.get("params"), dict) else {}
        self.keys(radar, RADAR_KEYS, RADAR_KEYS, "radar")
        self.keys(link, LINK_KEYS, LINK_KEYS, "link")
        self.keys(params, PARAM_KEYS, PARAM_KEYS, "params")
        fs = self.num(radar, "fs_hz", 0.5, 1000.0)
        start = self.num(radar, "range_start_m", 0.0, 1000.0)
        step = self.num(radar, "range_step_m", 1e-6, 1000.0)
        bins = self.num(radar, "bins", 1, MAX_BINS, integer=True)
        self.num(radar, "wavelength_mm", 0.1, 1e6)
        lost = self.num(link, "lost_frames", 0, integer=True)
        crc = self.num(link, "crc_errors", 0, integer=True)
        tim = self.num(link, "timing_flagged", 0, integer=True)
        stamped = self.boolean(link, "timestamps")
        if stamped is False and tim:
            self.bad("timing_flagged = %s although the front-end sends no timestamps" % tim)
        self.num(r, "duration_s", 0.0)
        self.num(r, "uptime_s", 0, integer=True)
        band = params.get("band_hz")
        if not (isinstance(band, list) and len(band) == 2 and all(isinstance(x, (int, float)) for x in band) and 0 < band[0] < band[1]):
            self.bad("params.band_hz is not [lo, hi]")
            band = None
        min_bin = self.num(params, "min_bin", 0, MAX_BINS, integer=True)
        low_sens = self.num(params, "low_sens_db", 0.0)
        for k in ("cpe", "require_imu"):
            self.boolean(params, k)
        for k in ("pfa", "pfc"):
            self.num(params, k, 1e-9, 0.5)
        if params.get("pfa") is not None and params.get("pfc") is not None and params["pfc"] < params["pfa"]:
            pass   # the confirmation stage is allowed to be looser than the detection stage
        # ---- segments
        total = self.num(r, "segments_total", 0, 64, integer=True)
        valid = self.num(r, "segments_valid", 0, 64, integer=True)
        invalid = self.num(r, "segments_invalid", 0, 64, integer=True)
        if None not in (total, valid, invalid) and valid + invalid != total:
            self.bad("segments_valid + segments_invalid (%d + %d) != segments_total (%d)" % (valid, invalid, total))
        segs = r.get("segments")
        if not isinstance(segs, list):
            self.bad("segments is not a list")
            segs = []
        if total is not None and len(segs) > total:
            self.bad("%d segment entries for segments_total = %d" % (len(segs), total))
        seg_reasons = []
        for i, s in enumerate(segs):
            if not isinstance(s, dict):
                self.bad("segments[%d] is not an object" % i)
                continue
            self.keys(s, SEG_KEYS, SEG_KEYS, "segments[%d]" % i)
            ok = self.boolean(s, "valid")
            if s.get("reason") not in REASONS:
                self.bad("segments[%d].reason %r unknown" % (i, s.get("reason")))
            elif ok is True and s["reason"] != "none":
                self.bad("segments[%d] is valid but has reason %r" % (i, s["reason"]))
            elif ok is False and s["reason"] == "none":
                self.bad("segments[%d] is invalid but has no reason" % i)
            seg_reasons.append(s.get("reason"))
            self.num(s, "bad_frac", 0.0, 1.0)
            for k in ("det", "platform", "untested", "best_bin"):
                self.num(s, k, 0, integer=True)
            for k in ("confirmed", "cpe", "vib"):
                self.boolean(s, k)
        if valid is not None and segs and total is not None and len(segs) == total:
            if sum(1 for s in segs if isinstance(s, dict) and s.get("valid") is True) != valid:
                self.bad("segments_valid = %d disagrees with the segment list" % valid)
        if "radar_timing" in seg_reasons and not tim:
            self.bad("a segment was rejected for radar_timing but timing_flagged is 0")

        # ---- baseline / sensitivity
        base_src, sens = r.get("baseline_src"), r.get("sensitivity")
        if base_src == "none":
            if sens != "unknown":
                self.bad("no baseline, yet sensitivity is %r (it can only be 'unknown')" % sens)
            if "baseline_db" in r or "floor_delta_db" in r:
                self.bad("baseline_db / floor_delta_db present without a baseline")
        else:
            if sens == "unknown":
                self.bad("a baseline exists (%s) but the sensitivity is 'unknown'" % base_src)
            if "baseline_db" not in r or "floor_delta_db" not in r:
                self.bad("baseline_src %r without baseline_db / floor_delta_db" % base_src)
        # "low" is a property of the measurement (the floor is raised), whatever the verdict. Only a negative
        # is downgraded because of it: NO_RELIABLE_PATTERN at a raised floor must not exist.
        delta = r.get("floor_delta_db")
        if isinstance(delta, (int, float)) and low_sens is not None:
            if sens == "low" and delta <= low_sens - 0.05:         # 0.05 dB: the record carries one decimal
                self.bad("sensitivity 'low' with floor_delta_db %.1f <= low_sens_db %.1f" % (delta, low_sens))
            if sens == "ok" and delta > low_sens + 0.05:
                self.bad("sensitivity 'ok' with floor_delta_db %.1f > low_sens_db %.1f" % (delta, low_sens))
        if verdict == "LOW_SENSITIVITY" and sens != "low":
            self.bad("LOW_SENSITIVITY with sensitivity %r" % sens)
        if verdict == "NO_RELIABLE_PATTERN" and sens == "low":
            self.bad("a negative at low sensitivity must be reported as LOW_SENSITIVITY, not NO_RELIABLE_PATTERN")

        # ---- verdict specifics
        has_line = bool(LINE_KEYS & set(r))
        if verdict in ("CANDIDATE_PATTERN", "POSSIBLE_PATTERN"):
            missing = LINE_KEYS - set(r)
            if missing:
                self.bad("%s without %s" % (verdict, ", ".join(sorted(missing))))
        elif verdict != "CONFOUNDED" and has_line:
            self.bad("%s must not carry candidate fields (%s)" % (verdict, ", ".join(sorted(LINE_KEYS & set(r)))))
        ev = r.get("evidence")
        if verdict == "CANDIDATE_PATTERN" and ev == "none":
            self.bad("a candidate with evidence 'none'")
        if verdict == "POSSIBLE_PATTERN" and ev != "weak":
            self.bad("a possible pattern must be 'weak' evidence, got %r" % ev)
        if verdict not in ("CANDIDATE_PATTERN", "POSSIBLE_PATTERN") and ev != "none":
            self.bad("%s with evidence %r" % (verdict, ev))
        reason = r.get("invalid_reason")
        if verdict == "INVALID_SCAN" and reason == "none":
            self.bad("INVALID_SCAN without a reason")
        if verdict in ("CANDIDATE_PATTERN", "POSSIBLE_PATTERN", "NO_RELIABLE_PATTERN", "LOW_SENSITIVITY") and valid is not None and valid < 1:
            self.bad("%s without a single valid segment" % verdict)
        if verdict == "CONFOUNDED":
            if not (r.get("vibration") is True or any(isinstance(s, dict) and s.get("platform", 0) > 0 for s in segs)):
                self.bad("CONFOUNDED without any sign of platform vibration")
        if verdict == "CANDIDATE_PATTERN":
            agree = self.num(r, "segments_agree", 2, 64, integer=True)
            if valid is not None and agree is not None and agree > valid:
                self.bad("segments_agree %d > segments_valid %d" % (agree, valid))
            if self.boolean(r, "vetted") is None:
                pass
        if has_line and verdict in ("CANDIDATE_PATTERN", "POSSIBLE_PATTERN"):
            b = self.num(r, "bin", 0, MAX_BINS, integer=True)
            hz = self.num(r, "hz", 0.0)
            bpm = self.num(r, "bpm", 0.0)
            rng = self.num(r, "range_m", 0.0)
            self.num(r, "snr_db", -100.0, 200.0)
            self.num(r, "margin_db", 0.0, 200.0)
            if None not in (hz, bpm) and abs(bpm - hz * 60.0) > 0.2:
                self.bad("bpm %.1f does not match hz %.3f x 60" % (bpm, hz))
            if band and hz is not None and not (band[0] - 1e-3 <= hz <= band[1] + 1e-3):
                self.bad("hz %.3f outside the respiration band %s" % (hz, band))
            if None not in (b, bins) and b >= bins:
                self.bad("bin %d beyond the %d range bins" % (b, bins))
            if None not in (b, min_bin) and b < min_bin:
                self.bad("bin %d below min_bin %d (static leakage zone)" % (b, min_bin))
            if None not in (b, start, step, rng) and abs(rng - (start + b * step)) > 0.006:
                self.bad("range_m %.2f does not match bin %d of the radar geometry (%.3f + bin x %.3f)" % (rng, b, start, step))
        if fs is not None and lost is not None and crc is not None:
            pass


def read_records(path):
    f = sys.stdin if path == "-" else open(path, "r", encoding="utf-8", errors="replace")
    try:
        for n, line in enumerate(f, 1):
            s = line.strip()
            if s.startswith("{"):
                yield n, s
    finally:
        if f is not sys.stdin:
            f.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--quiet", action="store_true", help="only print violations")
    ap.add_argument("--strict", action="store_true", help="warnings fail the check too")
    ap.add_argument("--min-records", type=int, default=1, help="fail if fewer records were found (default 1)")
    ap.add_argument("--ignore-ids", action="store_true", help="do not warn when scan ids do not increase (logs of several boots / units)")
    args = ap.parse_args(argv)

    chk = Checker()
    counts, total = {}, 0
    for path in args.files:
        try:
            recs = list(read_records(path))
        except OSError as e:
            print("cannot read %s: %s" % (path, e))
            return 1
        last_id = None
        for n, line in recs:
            chk.cur = "%s:%d" % (path, n)
            try:
                r = json.loads(line)
            except ValueError as e:
                chk.bad("not valid JSON (%s)" % e)
                continue
            if not isinstance(r, dict):
                chk.bad("not a JSON object")
                continue
            if "verdict" not in r:
                chk.warn("a JSON line without 'verdict' was skipped")
                continue
            total += 1
            counts[r.get("verdict")] = counts.get(r.get("verdict"), 0) + 1
            chk.cur = "%s:%d (%s)" % (path, n, r.get("scan_id"))
            chk.record(r)
            sid = r.get("scan_id")
            if not args.ignore_ids and isinstance(sid, str) and last_id is not None and sid <= last_id:
                chk.warn("scan id does not increase (%s after %s): a reboot or a log from another unit" % (sid, last_id))
            last_id = sid if isinstance(sid, str) else last_id
    if total < args.min_records:
        chk.violations.append("found %d scan record(s), expected at least %d" % (total, args.min_records))
    for w in chk.warnings:
        print("warning:", w)
    for v in chk.violations:
        print("VIOLATION:", v)
    if not args.quiet:
        print("%d scan record(s): %s" % (total, ", ".join("%s %d" % (k, counts[k]) for k in sorted(counts, key=str)) or "-"))
    bad = bool(chk.violations) or (args.strict and bool(chk.warnings))
    print("scan log %s: %d violation(s), %d warning(s)" % ("FAILED" if bad else "OK", len(chk.violations), len(chk.warnings)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
