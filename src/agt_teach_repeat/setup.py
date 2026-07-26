from glob import glob
from setuptools import find_packages, setup


package_name = "agt_teach_repeat"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="yangxuan",
    maintainer_email="yangxuan@example.com",
    description="Fail-closed teach, repeat, and repeatability evaluation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bag_path_extractor = agt_teach_repeat.bag_path_extractor:main",
            "teach_path_publisher = agt_teach_repeat.teach_path_publisher:main",
            "teach_path_validator = agt_teach_repeat.teach_path_validator:main",
            "teach_path_executor = agt_teach_repeat.teach_path_executor:main",
            "repeatability_evaluator = agt_teach_repeat.repeatability_evaluator:main",
            "corridor_auditor = agt_teach_repeat.corridor_auditor:main",
            "localization_map_evaluator = agt_teach_repeat.localization_map_evaluator:main",
        ]
    },
)
