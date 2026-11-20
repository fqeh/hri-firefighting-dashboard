from setuptools import setup

setup(
    name="hri_dashboard", version="0.1.0", packages=["hri_dashboard"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hri_dashboard"]),
        ("share/hri_dashboard", ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    tests_require=["pytest"],
    maintainer="HRI project contributors", maintainer_email="maintainers@example.invalid",
    description="Local ROS telemetry gateway", license="Apache-2.0",
    entry_points={"console_scripts": ["dashboard_server = hri_dashboard.server:main"]},
)
