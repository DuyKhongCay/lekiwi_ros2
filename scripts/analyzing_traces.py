"""Analyze ROS 2 callback durations from a tracetools trace."""

import argparse

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor import Processor
from tracetools_analysis.processor.ros2 import Ros2Handler


class PartialRos2Handler(Ros2Handler):
    """Process callback events when initialization events are absent."""

    @staticmethod
    def required_events():
        return set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "trace",
        nargs="?",
        default="/root/.ros/tracing/lekiwi_perception_trace",
    )
    args = parser.parse_args()

    events = load_file(args.trace)
    try:
        handler = Ros2Handler.process(events)
    except Processor.RequiredEventNotFoundError as exc:
        print(f"Warning: incomplete ROS 2 trace ({exc}); using callback-only mode")
        handler = PartialRos2Handler.process(events)

    callbacks = handler.data.callback_instances
    symbols = handler.data.callback_symbols
    symbol_by_object = {}
    if not symbols.empty:
        for symbol in symbols.reset_index().itertuples(index=False):
            symbol_by_object[symbol.callback_object] = symbol.symbol
    else:
        print("Warning: callback symbols are absent; showing callback addresses")

    if callbacks.empty:
        print("No completed callback instances found")
        return

    for callback in callbacks.itertuples(index=False):
        duration_ms = callback.duration.total_seconds() * 1000.0
        callback_name = symbol_by_object.get(
            callback.callback_object, f"callback@0x{int(callback.callback_object):x}"
        )
        print(
            f"Callback: {callback_name} [{callback.callback_object}], "
            f"Duration: {duration_ms:.3f} ms"
        )


if __name__ == "__main__":
    main()
