from nicolay_flow_meter_connector import NicolaySensor
import time

sensor = NicolaySensor(port="/dev/ttyUSB0")

call_count = 0
start_time = time.time()

while True:
    data = sensor.get_flow()
    call_count += 1
    elapsed_time = time.time() - start_time
    frequency = call_count / elapsed_time if elapsed_time > 0 else 0
    print(f"Frequency: {frequency:.2f} calls/sec", data)
