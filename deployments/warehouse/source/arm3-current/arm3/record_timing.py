"""Clock scheduling and capture timing reports; no hardware access."""
import math


def next_deadline(previous, now, period):
    """Advance an absolute schedule; skip missed slots without a catch-up burst."""
    deadline = previous + period
    if deadline <= now:
        deadline += (math.floor((now - deadline) / period) + 1) * period
    return deadline


def quality(records, fps):
    """Expose actual timing instead of inferring synchronization from FPS."""
    camera_names = ('wrist', 'overview')
    rows = []
    previous = None
    for row in records:
        stamps = row['camera_monotonic_s']
        read_intervals = row.get('timing', {}).get('joint_read_monotonic_s', {})
        if read_intervals:
            start = min(v[0] for v in read_intervals.values())
            end = max(v[1] for v in read_intervals.values())
            anchor = (start + end) / 2
            uncertainty = end - start
        else:
            anchor, uncertainty = row['monotonic_s'], None
        entry = {
            'source_index': row['index'], 'loop_monotonic_s': row['monotonic_s'],
            'state_reference_monotonic_s': anchor,
            'state_read_span_s': uncertainty,
            'camera_monotonic_s': stamps,
            'camera_skew_s': abs(stamps['wrist'] - stamps['overview']),
            'max_camera_state_offset_s': max(abs(stamps[c] - anchor) for c in camera_names),
            'repeated_control_sample': previous is not None and row['index'] == previous['index'],
            'repeated_cameras': {c: previous is not None and stamps[c] == previous['camera_monotonic_s'][c]
                                 for c in camera_names},
            'timing': row.get('timing', {}),
        }
        rows.append(entry)
        previous = row
    return {
        'schema_version': 1, 'clock': 'host_monotonic_seconds', 'fps': fps,
        'camera_timestamp_meaning': 'host receive/publication; not exposure synchronization',
        'frames': len(rows),
        'repeated_control_samples': sum(r['repeated_control_sample'] for r in rows),
        'repeated_cameras': {c: sum(r['repeated_cameras'][c] for r in rows) for c in camera_names},
        'camera_skew_over_frame': sum(r['camera_skew_s'] > 1/fps for r in rows),
        'camera_state_offset_over_frame': sum(r['max_camera_state_offset_s'] > 1/fps for r in rows),
        'rows': rows,
    }
