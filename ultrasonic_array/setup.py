from setuptools import find_packages, setup


package_name = "ultrasonic_array"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/lidar_proc.launch.py"]),
        (f"share/{package_name}/config", ["config/lidar_proc.yaml"]),
        (f"share/{package_name}/simulation", ["simulation/corridor.world"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AN Team",
    maintainer_email="autonomy@example.com",
    description="Near-field ultrasonic obstacle processing for Autonomous Navigation (AN).",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_proc = ultrasonic_array.lidar_proc_node:main",
            "sim_ultrasonic = ultrasonic_array.sim_ultrasonic_node:main",
        ],
    },
)
