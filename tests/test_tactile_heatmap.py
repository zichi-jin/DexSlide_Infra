import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from tactile_api import CMD_ADC_DATA, TactileCdcDecoder
from tactile_heatmap import (
    LiveFrame,
    TactileHeatmap,
    automatic_color_limits,
    format_cell_value,
    matrix_statistics,
    matrix_to_array,
    select_measurement_matrix,
)
from tactile_stream import RECORD_DATA, build_sensor_frame, encode_record


def decode_adc_measurement():
    rows = 2
    cols = 3
    data = (
        (123).to_bytes(4, "little")
        + bytes([rows, cols])
        + (8).to_bytes(2, "little")
        + (10).to_bytes(2, "little")
        + b"\xFF\xFF"
        + (12).to_bytes(2, "little")
        + (13).to_bytes(2, "little")
        + (14).to_bytes(2, "little")
        + (15).to_bytes(2, "little")
    )
    decoder = TactileCdcDecoder()
    events = decoder.feed(encode_record(RECORD_DATA, 4, build_sensor_frame(1, CMD_ADC_DATA, data)))
    return events[0].measurement


def test_auto_channel_selects_adc_and_masks_invalid_cells():
    measurement = decode_adc_measurement()
    assert measurement is not None

    selected = select_measurement_matrix(measurement, "auto")

    assert selected is not None
    matrix, label = selected
    assert label == "ADC counts"
    array = matrix_to_array(matrix)
    assert array.shape == (2, 3)
    assert np.isnan(array[0, 1])
    assert array[1, 2] == 15


def test_automatic_color_limits_cover_all_current_sensor_values():
    lower, upper = automatic_color_limits((np.array([[0.0, np.nan]]), np.array([[10.0]])))

    assert np.isclose(lower, -0.2)
    assert np.isclose(upper, 10.2)


def test_matrix_statistics_ignores_invalid_cells():
    statistics = matrix_statistics(np.array([[0.0, np.nan], [4.0, 9.0]]))

    assert statistics == (0, 9, 2)


def test_cell_value_formatting_preserves_raw_integer_values():
    assert format_cell_value(1234.0) == "1234"
    assert format_cell_value(np.nan) == "—"


def test_heatmap_keeps_the_latest_frame_for_its_connector():
    measurement = decode_adc_measurement()
    assert measurement is not None
    record = encode_record(RECORD_DATA, 4, measurement.raw_frame)
    dashboard = TactileHeatmap("adc", None, None)

    for offset in range(0, len(record), 7):
        dashboard.feed(record[offset:offset + 7])

    assert set(dashboard.frames) == {4}
    assert dashboard.frames[4].sequence == 8
    assert dashboard.frames[4].matrix.shape == (2, 3)


def test_heatmap_locks_one_shared_colour_range_after_warmup():
    dashboard = TactileHeatmap("adc", None, None)
    dashboard.frames = {
        0: LiveFrame(np.array([[10.0, 20.0]]), "ADC counts", 1, 0.0),
        1: LiveFrame(np.array([[30.0, np.nan]]), "ADC counts", 1, 0.0),
    }

    limits = dashboard.configure_color_limits()

    assert np.isclose(limits[0], 9.6)
    assert np.isclose(limits[1], 30.4)


def test_heatmap_blit_artists_all_belong_to_an_axes():
    dashboard = TactileHeatmap("adc", None, None)
    dashboard.create_figure()

    assert all(artist.axes is not None for artist in dashboard.initial_artists())


def test_annotated_heatmap_creates_all_11_by_12_by_8_cell_labels():
    dashboard = TactileHeatmap("adc", None, None, annotate=True)
    dashboard.create_figure()

    assert sum(len(port_texts) for port_texts in dashboard._value_texts) == 11 * 12 * 8


def test_sparse_annotation_only_draws_nonzero_cells():
    dashboard = TactileHeatmap("adc", None, None, annotate=True)
    dashboard.create_figure()

    changed = dashboard._update_value_texts(0, np.array([[0.0, 12.0], [0.0, np.nan]]))

    assert len(changed) == 1
    assert dashboard._value_texts[0][0].get_text() == ""
    assert dashboard._value_texts[0][1].get_text() == "12"
