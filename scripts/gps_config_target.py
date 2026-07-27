"""Expose the GPS receiver configuration as a PlatformIO task.

Registers a custom target so the receiver can be set up from the Tasks UI or
with ``pio run -e gps_config -t configure``. The task talks to the receiver over
its USB port and builds no firmware.
"""

Import("env")

env.AddCustomTarget(
    name="configure",
    dependencies=None,
    actions=['"$PYTHONEXE" "$PROJECT_DIR/scripts/configure_gps.py"'],
    title="GPS konfigurieren",
    description="NEO-M8T auf 10 Hz, 115200 Baud und nur GGA+RMC einstellen",
    always_build=True,
)
