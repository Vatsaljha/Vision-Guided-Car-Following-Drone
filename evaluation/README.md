# Evaluation protocol

## Detection labels

Use one row per sampled frame. For each frame, record whether roof marker ID 7 is truly visible and reasonably detectable, then record whether the detector reported a detection for that same frame. Use a predefined sampling interval (for example, every Nth received frame) and include straight segments, both directions of the figure-eight, near-edge frames, and representative lighting/distance conditions. Avoid selecting only easy frames.

`ground_truth_id7=1` means a positive frame under the protocol; `0` means a negative frame. `predicted_id7` is the binary detector output. Do not mark an off-screen marker as a false negative if the evaluation definition says the marker must be in view to be detectable. Record this rule before scoring and apply it consistently.

The reported accuracy, precision, recall, specificity and F1 are frame-level metrics, not a guarantee that the full drone-following system succeeds.

## End-to-end trials

Use one row per complete trial and preserve failures. Record planned and completed laps plus the amount of time the drone has an eligible opportunity to track the car. `tracking_seconds` should count time during which tracking was valid under your protocol; `eligible_tracking_seconds` is the total evaluation time during which tracking was expected. For every marker-loss event, measure the time to reacquisition; record their sum in `total_recovery_seconds`.

Report test date, hardware, Gazebo real-time factor, camera rate, marker size/distance, number of trials and any manual interventions in the project notes/README. Do not call the successful-trial percentage “detection accuracy.”
