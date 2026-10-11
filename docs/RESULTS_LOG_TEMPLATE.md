# Experiment results log (template)

Copy this file for each measured release or test session. Do not fill the result cells with estimates.

## Test metadata

- Date/time:
- Git commit:
- OS / ROS 2 / Gazebo version:
- CPU / RAM / GPU:
- Test world and launch sequence:
- Number of labeled images:
- Number of independent simulation trials:
- Image sampling method:
- Ground-truth labeling rule:
- Marker distance / apparent pixel size range:
- Lighting / occlusion / turn segments included:
- Gazebo real-time factor range:
- Camera image topic measured rate:
- Any manual interventions:

## Detection

| Metric | Result |
|---|---:|
| TP | |
| FP | |
| TN | |
| FN | |
| Accuracy (%) | |
| Precision (%) | |
| Recall (%) | |
| Specificity (%) | |
| F1 (%) | |

Command and data file used:

```text
python3 tools/evaluate_detection.py evaluation/detection_labels.csv --json evaluation/detection_results.json
```

## End-to-end following

| Metric | Result |
|---|---:|
| Trials | |
| Trials completing all planned laps | |
| Successful-trial rate (%) | |
| Planned laps | |
| Completed laps | |
| Lap completion rate (%) | |
| Tracking availability (%) | |
| Marker-loss events | |
| Mean recovery time (s) | |

Command and data file used:

```text
python3 tools/evaluate_trials.py evaluation/trial_results.csv --json evaluation/trial_results.json
```

## Observations and failures

- Conditions when tracking failed:
- Detection flicker / false detections:
- Turns completed / turns missed:
- Notes on camera lag or simulation speed:
- Changes since the previous test:

## Interpretation

State what the results support and what remains untested. Do not call frame-level detector accuracy the same thing as full mission success.
