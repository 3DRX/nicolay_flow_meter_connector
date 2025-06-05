from setuptools import setup, Extension
import sys

c_src_files = [
    "nicolay_python_binding.c",
    "nicolay_flow_sensor.c",
]
extra_link_args = []

if sys.platform.startswith("linux") or sys.platform == "darwin":
    c_src_files.append("platform_serial_posix.c")
elif sys.platform == "win32":
    print("Windows is currently not supported")
    pass
else:
    print("Unknown platform, not supported")
    pass


nicolay_sensor_module = Extension(
    "nicolay_flow_meter_connector",
    sources=c_src_files,
    extra_link_args=extra_link_args,
)

setup(
    name="nicolay-flow-meter-connector",
    version="0.1.0",
    author="Jingyang Kang",
    author_email="3drxkjy@gmail.com",
    description="Nicolay Flow Meter Connection SDK",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/3DRX/nicolay_flow_meter_connector",
    ext_modules=[nicolay_sensor_module],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: C",
        "Operating System :: OS Independent",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Scientific/Engineering",
    ],
    python_requires=">=3.8",
)
