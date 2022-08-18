from glob import glob

from setuptools import setup

package_name = "flexiv_test_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + "/configs", glob("configs/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Mun Seng Phoon",
    author_email="munseng.phoon@flexiv.com",
    maintainer="Mun Seng Phoon",
    maintainer_email="munseng.phoon@flexiv.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Demo nodes for testing flexiv_ros2.",
    long_description="""\
Demo nodes for for testing flexiv_ros2.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publisher_joint_trajectory_controller = \
                flexiv_test_nodes.publisher_joint_trajectory_controller:main",
            "sine_sweep_position_controller = \
                flexiv_test_nodes.sine_sweep_position_controller:main",
            "sine_sweep_impedance_controller = \
                flexiv_test_nodes.sine_sweep_impedance_controller:main",
        ],
    },
)
