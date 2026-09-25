# `data/` — where to put a Scenario Export next to the application

This folder is empty on purpose. It is the first place the display looks for the simulation's
exported files when they are not configured elsewhere:

```
data/
└── Scenario_Export/
    ├── Sensor_Properties/
    │   └── sensor_properties.json
    ├── 3D Trajectory/
    │   ├── Ownship_Primary_Radar.csv
    │   ├── Ownship_Secondary_Radar.csv
    │   └── target_<TgtId>.csv
    └── Scan_Scheduler/
        ├── Scan_Scheduler_Primary_Radar.csv
        └── Scan_Scheduler_Secondary_Radar.csv
```

Copy or symlink a Scenario Export here and the display will find it with no configuration at
all. You do not have to: the export can stay wherever the simulation wrote it and be selected
with any of

```
run_windows.bat --export "C:\path\to\Scenario_Export"
./run_linux.sh --export /path/to/Scenario_Export
MSDF_SCENARIO_EXPORT=/path/to/Scenario_Export ./run_linux.sh
```

or by editing `scenario_export.root` in `config/display_config.json` (a relative path is resolved
from the project folder, so the configuration stays portable).

**These files are inputs.** The display opens them read-only and never writes to, moves or
deletes anything in a Scenario Export. Derived arrays are cached separately, in `cache/`.

There is no sample data in this project: a Scenario Export is real recorded radar data, and
inventing a stand-in would mean showing numbers that never came from the simulation. Generate one
with `Moving_Radar.ipynb`, or point the display at an existing export.
