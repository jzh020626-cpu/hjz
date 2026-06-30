from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
FORCE_MONITOR = REPO_ROOT / "src/wing_alignment_sensing/src/force_monitor.cpp"
LAUNCH_BUILDERS = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/launch_builders.py"


def test_force_monitor_tracks_invalid_streak_and_health_policy():
    text = FORCE_MONITOR.read_text(encoding="utf-8")

    assert 'invalid_streak_warn' in text
    assert 'invalid_streak_unavailable' in text
    assert 'log_throttle_sec' in text
    assert 'hold_last_valid_filtered' in text
    assert 'invalid_sample_count_' in text
    assert 'invalid_streak_' in text
    assert 'FORCE_HEALTH][UNAVAILABLE' in text


def test_force_monitor_invalid_samples_do_not_update_filter_or_contact_freshness():
    text = FORCE_MONITOR.read_text(encoding="utf-8")

    assert 'if (!valid_force_sample_(raw))' in text
    assert 'return;' in text
    assert 'last_force_msg_time_ = now();' in text
    invalid_branch = text.split('if (!valid_force_sample_(raw)) {', 1)[1].split('have_last_valid_raw_ = true;', 1)[0]
    assert 'last_force_msg_time_ = now();' not in invalid_branch
    assert 'filters_[i].update' not in invalid_branch


def test_force_monitor_does_not_publish_stale_filtered_force_when_health_is_unavailable():
    text = FORCE_MONITOR.read_text(encoding="utf-8")

    assert 'timer_filtered_cb_' in text
    assert 'pub_filtered_->publish(m);' in text
    assert 'force_data_unavailable_' in text or 'invalid_streak_ >= invalid_streak_unavailable_' in text


def test_force_monitor_launch_reads_shared_config_file():
    text = LAUNCH_BUILDERS.read_text(encoding="utf-8")

    force_monitor_section = text.split("executable='force_monitor'", 1)[1].split("ros_arguments=['--log-level', 'WARN']", 1)[0]
    assert "parameters=[config_file]" in force_monitor_section or "parameters=[config_file, {" in force_monitor_section
