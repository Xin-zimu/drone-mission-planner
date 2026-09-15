from setuptools import find_packages, setup

package_name = "dmp_crazyflie_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/bridge.yaml", "config/crazyflies.yaml.example"]),
        (f"share/{package_name}/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Drone Mission Planner",
    maintainer_email="maintainer@example.com",
    description="Drone Mission Planner Crazyflie execution bridge skeleton.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dmp-crazyflie-bridge=dmp_crazyflie_bridge.protocol_server:main",
        ],
    },
)

