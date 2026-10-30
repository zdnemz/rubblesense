# RubbleSense Monte-Carlo (SIMULATION ONLY)

Parameters: pfa_segment=0.01 pfa_confirm=0.02 band=0.10-0.80 Hz cfar=2+8 guard/ref cpe=on
Mode: full

### E1 - Empty scene (no person), 20000 scans

| Verdict | Scans | Share (95 % CI) |
|---|---|---|
| CANDIDATE_PATTERN | 2 | 0.01 (0.00-0.04) % |
| POSSIBLE_PATTERN | 291 | 1.46 (1.29-1.63) % |
| CONFOUNDED | 0 | 0.00 (0.00-0.02) % |
| LOW_SENSITIVITY | 0 | 0.00 (0.00-0.02) % |
| INVALID_SCAN | 0 | 0.00 (0.00-0.02) % |
| NO_RELIABLE_PATTERN | 19707 | 98.53 (98.36-98.70) % |

First-segment CFAR alarm rate (design: pfa_segment = 1.0 %): 0.85 (0.73-0.99) %
Mean scan duration: 51.6 s

### E2 - Person breathing at 16.8 /min, detection vs. reflection amplitude (400 scans each)

| amplitude (counts) | nominal tone SNR (dB) | CANDIDATE correct | CANDIDATE any | POSSIBLE | NO_PATTERN | median rate error (/min) | mean scan (s) |
|---|---|---|---|---|---|---|---|
| 6 | 7.6 | 12.50 (9.42-16.15) % | 12.5 % | 7.0 % | 80.5 % | 0.37 | 55 |
| 9 | 11.1 | 86.50 (82.76-89.69) % | 86.5 % | 1.2 % | 12.2 % | 0.37 | 58 |
| 12 | 13.6 | 99.50 (98.21-99.94) % | 99.5 % | 0.0 % | 0.5 % | 0.34 | 52 |
| 15 | 15.5 | 100.00 (99.08-100.00) % | 100.0 % | 0.0 % | 0.0 % | 0.34 | 51 |
| 18 | 17.1 | 100.00 (99.08-100.00) % | 100.0 % | 0.0 % | 0.0 % | 0.34 | 51 |
| 21 | 18.4 | 100.00 (99.08-100.00) % | 100.0 % | 0.0 % | 0.0 % | 0.29 | 51 |
| 24 | 19.6 | 100.00 (99.08-100.00) % | 100.0 % | 0.0 % | 0.0 % | 0.33 | 51 |
| 30 | 21.5 | 100.00 (99.08-100.00) % | 100.0 % | 0.0 % | 0.0 % | 0.31 | 51 |
| 40 | 24.0 | 100.00 (99.08-100.00) % | 100.0 % | 0.0 % | 0.0 % | 0.30 | 51 |
| 60 | 27.6 | 100.00 (99.08-100.00) % | 100.0 % | 0.0 % | 0.0 % | 0.35 | 51 |

### E3 - Platform micro-vibration the IMU cannot see, NO person (2000 scans each, random frequency 0.15-0.75 Hz)

| vibration (mm) | compensation | CANDIDATE | POSSIBLE | CONFOUNDED | NO_PATTERN | vibration notice shown |
|---|---|---|---|---|---|---|
| 0.00 | on | 0.00 (0.00-0.18) % | 1.3 % | 0.0 % | 98.7 % | 2.4 % |
| 0.05 | on | 0.00 (0.00-0.18) % | 0.8 % | 0.4 % | 98.8 % | 100.0 % |
| 0.10 | on | 0.05 (0.00-0.28) % | 1.0 % | 0.2 % | 98.8 % | 100.0 % |
| 0.20 | on | 0.00 (0.00-0.18) % | 0.6 % | 0.2 % | 99.2 % | 100.0 % |
| 0.40 | on | 0.00 (0.00-0.18) % | 0.8 % | 0.2 % | 99.0 % | 100.0 % |
| 0.00 | off | 0.00 (0.00-0.18) % | 1.1 % | 0.0 % | 99.0 % | 2.1 % |
| 0.05 | off | 0.00 (0.00-0.18) % | 0.2 % | 99.8 % | 0.0 % | 100.0 % |
| 0.10 | off | 0.00 (0.00-0.18) % | 0.1 % | 100.0 % | 0.0 % | 100.0 % |
| 0.20 | off | 0.00 (0.00-0.18) % | 0.1 % | 99.9 % | 0.0 % | 100.0 % |
| 0.40 | off | 0.00 (0.00-0.18) % | 0.1 % | 99.9 % | 0.0 % | 100.0 % |

### E4 - Person (amplitude 30) AND platform vibration of 0.2 mm (500 scans each)

| vibration frequency | CANDIDATE correct | POSSIBLE | CONFOUNDED | NO_PATTERN | vibration notice |
|---|---|---|---|---|---|
| none | 100.00 (99.26-100.00) % | 0.0 % | 0.0 % | 0.0 % | 2.6 % |
| 0.15 Hz | 100.00 (99.26-100.00) % | 0.0 % | 0.0 % | 0.0 % | 100.0 % |
| 0.28 Hz | 0.80 (0.22-2.04) % | 2.0 % | 97.0 % | 0.0 % | 100.0 % |
| 0.45 Hz | 100.00 (99.26-100.00) % | 0.0 % | 0.0 % | 0.0 % | 100.0 % |
| 0.70 Hz | 100.00 (99.26-100.00) % | 0.0 % | 0.0 % | 0.0 % | 100.0 % |

### E5 - Impulsive bumps of the sensor (rate 0.3 /s), with and without a person (500 scans each)

| bump size (mm) | person | INVALID_SCAN | CANDIDATE correct | CANDIDATE wrong | POSSIBLE | CONFOUNDED | NO_PATTERN |
|---|---|---|---|---|---|---|---|
| 0.05 | no | 0.0 % | - % | 0.20 (0.01-1.11) % | 2.0 % | 0.0 % | 97.8 % |
| 0.10 | no | 0.0 % | - % | 0.20 (0.01-1.11) % | 2.0 % | 0.0 % | 97.8 % |
| 0.30 | no | 7.2 % | - % | 0.00 (0.00-0.74) % | 2.6 % | 0.0 % | 90.2 % |
| 1.00 | no | 97.4 % | - % | 0.00 (0.00-0.74) % | 0.0 % | 0.0 % | 2.6 % |
| 0.05 | yes | 0.0 % | 100.0 % | 0.00 (0.00-0.74) % | 0.0 % | 0.0 % | 0.0 % |
| 0.10 | yes | 0.0 % | 100.0 % | 0.00 (0.00-0.74) % | 0.0 % | 0.0 % | 0.0 % |
| 0.30 | yes | 3.8 % | 90.6 % | 0.00 (0.00-0.74) % | 5.6 % | 0.0 % | 0.0 % |
| 1.00 | yes | 83.0 % | 3.2 % | 0.00 (0.00-0.74) % | 13.8 % | 0.0 % | 0.0 % |

### E6 - Noisier-than-bench conditions: the negative result must lose its meaning (500 scans each)

Baseline floor 21.1 dB, LOW_SENSITIVITY above +3 dB. Empty scene and a person of amplitude 30.

| noise sigma (counts) | floor over baseline (dB) | empty: LOW_SENSITIVITY | empty: NO_PATTERN | person: CANDIDATE correct | person: LOW_SENSITIVITY | person: NO_PATTERN |
|---|---|---|---|---|---|---|
| 8 | +0.1 | 0.0 % | 99.0 % | 100.0 % | 0.0 % | 0.0 % |
| 11 | +2.8 | 22.0 % | 76.4 % | 100.0 % | 0.0 % | 0.0 % |
| 16 | +6.1 | 98.0 % | 0.0 % | 100.0 % | 0.0 % | 0.0 % |
| 24 | +9.6 | 99.0 % | 0.0 % | 95.4 % | 3.8 % | 0.0 % |
| 32 | +12.1 | 99.4 % | 0.0 % | 42.4 % | 51.0 % | 0.0 % |

### E7 - Rate and range accuracy of confirmed candidates (amplitude 40, 300 scans per rate)

| true rate (/min) | CANDIDATE correct | median rate error (/min) | 90th pct error (/min) | bin within +-1 |
|---|---|---|---|---|
| 7.0 | 100.0 % | 0.13 | 0.32 | 100.0 % |
| 10.0 | 100.0 % | 0.19 | 0.43 | 100.0 % |
| 16.8 | 100.0 % | 0.31 | 0.70 | 100.0 % |
| 24.0 | 100.0 % | 0.52 | 1.03 | 100.0 % |
| 36.0 | 96.7 % | 0.62 | 1.55 | 100.0 % |
| 45.0 | 89.3 % | 0.88 | 2.01 | 100.0 % |

### E8 - Vibration that is NOT perfectly common to all range bins (1000 scans each, 0.2 mm, random frequency 0.15-0.75 Hz)

Each bin sees the platform phase scaled by (1 + m*g), g ~ N(0,1). m = 0 is the idealised simulator; real scenes are not.

| mismatch m | person | CANDIDATE correct | CANDIDATE false | POSSIBLE | CONFOUNDED | NO_PATTERN |
|---|---|---|---|---|---|---|
| 0.0 | no | - | 0.00 (0.00-0.37) % | 0.8 % | 0.1 % | 99.1 % |
| 0.1 | no | - | 0.50 (0.16-1.16) % | 0.4 % | 94.1 % | 5.0 % |
| 0.3 | no | - | 0.10 (0.00-0.56) % | 0.2 % | 99.7 % | 0.0 % |
| 0.5 | no | - | 0.10 (0.00-0.56) % | 0.6 % | 99.3 % | 0.0 % |
| 0.0 | yes | 83.2 % | 0.00 (0.00-0.37) % | 0.8 % | 16.0 % | 0.0 % |
| 0.1 | yes | 84.6 % | 0.00 (0.00-0.37) % | 0.4 % | 15.0 % | 0.0 % |
| 0.3 | yes | 75.2 % | 0.90 (0.41-1.70) % | 2.1 % | 21.8 % | 0.0 % |
| 0.5 | yes | 66.6 % | 0.40 (0.11-1.02) % | 2.5 % | 30.5 % | 0.0 % |
