"""Tests for the scout base protocol parsing and formatting helpers."""

from scout_base_py.protocol import (
    Ack,
    format_drive,
    format_servo,
    ImuSample,
    parse_line,
    WatchdogStop,
)


def test_valid_parse_line():
    """Test parsing a valid IMU line."""
    imu_line = 'I 123 0.1 0.2 0.3 0.4 0.5 0.6 45.0 90 12.5'
    imu_sample = parse_line(imu_line)
    assert isinstance(imu_sample, ImuSample)
    assert isinstance(imu_sample.time_ms, int)
    assert isinstance(imu_sample.accel_x, float)
    assert isinstance(imu_sample.accel_y, float)
    assert isinstance(imu_sample.accel_z, float)
    assert isinstance(imu_sample.gyro_x, float)
    assert isinstance(imu_sample.gyro_y, float)
    assert isinstance(imu_sample.gyro_z, float)
    assert isinstance(imu_sample.yaw_deg, float)
    assert isinstance(imu_sample.servo_deg, int)
    assert isinstance(imu_sample.batt_voltage, float)


def test_ack_parse_line():
    """Test parsing an acknowledgement line."""
    ack_line_ok = 'A S ok'
    ack_line_ready = 'A boot ready'
    ack = parse_line(ack_line_ok)
    assert isinstance(ack, Ack)
    assert ack.text == 'S ok'

    ack = parse_line(ack_line_ready)
    assert isinstance(ack, Ack)
    assert ack.text == 'boot ready'


def test_watchdog_stop_parse_line():
    """Test parsing a watchdog stop line."""
    # Test WatchdogStop line
    watchdog_line = 'W stop'
    watchdog_stop = parse_line(watchdog_line)
    assert isinstance(watchdog_stop, WatchdogStop)


def test_parse_line_invalid():
    """Test that invalid protocol lines return None."""
    # Test invalid line
    empty_line = ''
    short_line = 'I'
    none_number_line = 'I 123 abc 0.2 0.3 0.4 0.5 0.6 45.0 90 12.5'
    truncated_line = 'I 123 0.1 0.2 0.3 0.4 0.5 0.6 45.0 90'
    echoed_command_line = 'M 1 2'
    two_hundred_random_characters_line = 'I ' + ' '.join(['0.1'] * 200)
    invalid_lines = [
        short_line,
        none_number_line,
        empty_line,
        truncated_line,
        echoed_command_line,
        two_hundred_random_characters_line,
    ]
    for line in invalid_lines:
        invalid_sample = parse_line(line)
        assert invalid_sample is None


def test_format_drive():
    """Test formatting and clamping drive commands."""
    assert format_drive(100, 200) == 'M 100 200\n'
    assert format_drive(-300, 300) == 'M -255 255\n'
    assert format_drive(0, 0) == 'M 0 0\n'


def test_format_servo():
    """Test formatting and clamping servo commands."""
    assert format_servo(90) == 'V 90\n'
    assert format_servo(-10) == 'V 0\n'
    assert format_servo(200) == 'V 180\n'
