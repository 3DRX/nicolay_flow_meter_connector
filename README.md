# nicolay_flow_meter_connector
Unofficial SDK for Nicolay flow meter connector

## Quick Start

```sh
pip install git+https://github.com/3DRX/nicolay_flow_meter_connector.git
```

```python
from nicolay_flow_meter_connector import NicolaySensor

sensor = NicolaySensor(port="/dev/ttyUSB0")

while True:
    data = sensor.get_flow()
    print(f"{data} mSlm^2")
```

## Supported Hardware

See https://nicolay.de/wp-content/uploads/sites/2/2021/02/NICOLAY-Datasheet-Flow-Meter-Connector-2021.pdf
